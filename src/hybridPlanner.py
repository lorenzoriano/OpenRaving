from PDDLPatcher import *
import sys, subprocess, time, os
import planning_primitives
from settings import *
import getopt
import StringIO
import glob
import utils
import re

totalExecTime = 0
#myPatcher = None

def execCmd(cmd,  successStr, outputFname, pollTime = 2):
    ''' A generic utility for executing a command. 
    Dumpfile stores stdout and stderr'''
    initTime = time.time()
    print "Executing %s...   " %(cmd), 
    sys.stdout.flush()
    ## Using subprocess.call so that all the messages from the planner
    ## can be captured and parsed for the string "Solution found!"

    dumpF = open(outputFname,  "w")
    p = subprocess.Popen([cmd], shell = True, stdout=dumpF, stderr=dumpF)
    startTime = time.time()
    #p = subprocess.Popen([cmd], shell = True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    while p.poll() == None:
        time.sleep(pollTime)
        if time.time() - startTime >350:
            #ans = raw_input("Still running. Continue (c), Restart (r), Quit(q)? ")
            ans = "q"
	    print "Killing process" 
	    startTime = time.time()
            if ans.strip() == "q":
                os.kill(p.pid+1,  9)
                sys.exit(-1)
            elif ans.strip() == "c":
                continue
            elif ans.strip() == "r":
                os.kill(p.pid+1,  9)
                return execCmd(cmd, successStr)
            else:
                print "Unrecognized response. Continuing."

    dumpF.close()
    time.sleep(5)
    endTime = time.time()

    dumpF = open(outputFname, "r")
    msg = dumpF.read()
    #out, err = p.communicate()
    #msg = out+err
    
    print "\nPlanning time: {0}".format(endTime-initTime)
    global totalExecTime
    totalExecTime +=  endTime-initTime

    if successStr in msg:
        print "Success!"
        print
        return msg
    else:
        print "Failure... Planner message:"
        print msg
        return -1

def runPlannerFF(pddlDomainFile, pddlProblemFile, ffOutputFile):
    ffCmdLine = ff + " -o " + pddlDomainFile +" -f " + pddlProblemFile
    for fileName in glob.glob(ffOutputFile):
        os.remove(fileName)
        
    retVal = execCmd(ffCmdLine, "found legal plan", ffOutputFile)
    if retVal ==-1:
        return -1,-1
        
    #tryIO(ffOutputFile, "write", retVal)
    
    ffOutStr = OutputParser(ffOutputFile).getFFPlan()
    return ffOutStr, 1


def runPlannerFD(pddlDomainFile, pddlProblemFile, fdOutputFName="fdOutput", \
                     pollTime=30, planQuality=4, TimeLimit=10000):
    '''fdOutputFName.X only gets the output plan. messages go to fdOutputFName '''
    fdCmdLine = fd + " " + pddlDomainFile +"  " + pddlProblemFile \
                         + " "+ fdOutputFName
    ext = "."+repr(planQuality)
    for fileName in glob.glob(fdOutputFName+".*"):
        os.remove(fileName)

    p = subprocess.Popen([fdCmdLine], shell = True, \
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print "Running " + fdCmdLine
    startTime = time.time()

    while p.poll() == None:
        print "Available plans: "+ repr(glob.glob(fdOutputFName+".*"))
        time.sleep(pollTime)
        if (os.path.exists(fdOutputFName+ext)  or \
                ((time.time() - startTime) > TimeLimit)):
            killproc = subprocess.Popen(["pkill -KILL downward"], shell = True)
            print "killing planning process"
            print "Available plans: "+ repr(glob.glob(fdOutputFName+".*"))
            break
    endTime = time.time()

    out, err = p.communicate()
    msg = out+err
    
    print "\nPlanning time: {0}".format(endTime-startTime)
    global totalExecTime
    totalExecTime +=  endTime-startTime

    if "Solution found" in msg:
        print "Success!"
    else:
        print "Failure... Planner message:"
        print msg
        return -1,-1

    tryIO(fdOutputFName, "write", msg)

    planCount = len(glob.glob(fdOutputFName+".*"))
    bestExt = "."+repr(planCount)
    print "\n\n\n"
    print "Using plan from "+ fdOutputFName+bestExt
    fdPlanStr = tryIO(fdOutputFName+bestExt, "read")
    
    return fdPlanStr, msg, planCount





def runPlanner(pddlDomainFile, pddlProblemFile, outputFname, plannerName="ff"):
    planStr = ""
    plannerOutput = ""
    if plannerName == "ff":
        planStr, planCount = runPlannerFF(pddlDomainFile, pddlProblemFile, outputFname)
        if planStr == -1:
            return -1,-1,-1
    elif plannerName == "fd":
        fdPlanStr, plannerOutput, planCount = runPlannerFD(pddlDomainFile, \
                                                           pddlProblemFile, outputFname)
        if fdPlanStr == -1:
            return -1, -1, -1
        # modify planStr to add line numbers etc.
        planLines = fdPlanStr.replace("(", "").replace(")","").split("\n")
        for lineNum in range(0,len(planLines)):
            if planLines[lineNum].strip()!="":
                planStr += "\t"+repr(lineNum)+": "+  planLines[lineNum].upper() + "\n"
        
    print "Computed plan: \n" + planStr
    strPlanFileH = StringIO.StringIO()
    strPlanFileH.write(planStr)
    strPlanFileH.seek(0)
    
    ##TBD!!: make both planners return same output in 1 msg/file    
    return strPlanFileH, plannerOutput, planCount


def updateInitFile(myPatcher, pddlProblemFile, iteration, plannerOutFname, \
                   plannerOutStr, errorStr, planCount, planner = "ff"):
    errorLines = errorStr.lower().split("\n")
    failedActionNumber = int(errorLines[0].strip("linenumber: "))
    propList = filter(lambda x:len(x) > 0, errorLines[1:])
## If action k failed, need state just before after action k
## kth element of state list from ffoutputfile is
## the state before application of action k.
    if planner == "ff":
        myPatcher.patchWithFFOutput(plannerOutFname, failedActionNumber)
    elif planner == "fd":
        myPatcher.patchWithFDOutput(plannerOutStr, failedActionNumber, planCount)
        
    myPatcher.patchWithProps(propList)
    print "State to re-plan from:"
    myPatcher.printInitState()
    myPatcher.writeCurrentInitState(pddlProblemFile)


def forgetAndRestart(myPatcher, learnedPredicate, problemFile, executor, cacheClearCount):
    cacheClearCount += 1
    myPatcher.forgetLearnedFactsAbout("obstructs")
    myPatcher.writeCurrentInitState(problemFile)
    planning_primitives.clearGPCache(executor)
    return cacheClearCount


def terminateOrReinterpret():
    reinterpret = raw_input("Planner failed. Reinterpret (y/n)?")
    if reinterpret != 'y':
        print "Quittting"
        sys.exit(-1)


    return True
      
            
def getObjSeqInPlan(file_object_or_name, objectNamePrefix = 'object'):
        objSeq = []
        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
            
        planStr = file_obj.read().lower()
        planLines = planStr.strip().split("\n")
        for line in planLines:
            if 'grasp' in line:
                fixedWLine = re.sub(r' +', ' ', line)
                objMatch = re.search('(?<=grasp )\w+', fixedWLine)
                obj = objMatch.group(0)
                objSeq.append(obj)
                
        file_obj.seek(0)
    
        return objSeq       

def iterativePlanAuto(myPatcher, ex, pddlDomainFile, pddlProblemFile, viewer, envFile, planner = "ff"):
    iteration = 0
    initialProblemFile = pddlProblemFile
    cacheClearCount = 0
    prevPDDLFile = None
    while True:
        reinterpreted = False
        iteration += 1
        plannerOutFname = pddlProblemFile+ ".out"
        planCount = 0
        strPlanFileH, plannerOutStr, planCount = runPlanner(pddlDomainFile, pddlProblemFile, plannerOutFname, planner)
 
        if strPlanFileH ==-1:
            reinterpreted = terminateOrReinterpret()
            cacheClearCount = forgetAndRestart(myPatcher, "obstructs", pddlProblemFile, ex, \
                cacheClearCount)
            continue

        objSeq = []
        if planner == "ff":
            objSeq = getObjSeqInPlan(strPlanFileH)

        try:
            planning_primitives.test(strPlanFileH, ex, objSeq)
            print "Success. Quitting."
            print "Total planning time: {0}".format(totalExecTime)
            print "Replan count: "+ repr(iteration)
            print "Cache clearing count: "+ repr(cacheClearCount)
            if prevPDDLFile is not None:
                obsCount = tryIO(prevPDDLFile, "read").count("obstructs")
            else:
                obsCount = 0
            print "Number of obstructions handled: "+ repr(obsCount)
            sys.exit(0)
        except planning_primitives.ExecutingException, e:
            errorStr = e.pddl_error_info
    
        if errorStr == "":
            print "Lower level failed without error message. Possibly due to sampling limit."
            reinterpreted = terminateOrReinterpret()
            cacheClearCount = forgetAndRestart(myPatcher, "obstructs", pddlProblemFile, ex, \
                cacheClearCount)
            continue

        print "Got facts:"
        print errorStr
        if viewer:
            #raw_input("Press return to continue")
            time.sleep(0.5)
        
        prevPDDLFile = pddlProblemFile
        pddlProblemFile = initialProblemFile.replace(".pddl", repr(iteration) + ".pddl")

            
        updateInitFile(myPatcher, pddlProblemFile, iteration, plannerOutFname, \
                       plannerOutStr, errorStr, planCount, planner)
        print

def run_with_ros(detector_and_cluster_map, envFile, viewer=True):
    planning_primitives.use_ros = True
    planning_primitives.detector_and_cluster_map = detector_and_cluster_map
    setupAndStart(pddlDomainFile, initialProblemFile,
                      viewer, envFile, planner="ff")

def setupAndStart(pddlDomainFile, initialProblemFile,\
    viewer, envFile, planner="ff"):
    editedProblemFile = initialProblemFile
    iteration = 0
    
    print "Using initial problem file " + editedProblemFile
    ORSetup = planning_primitives.initOpenRave(viewer, envFile)
    goalObject = raw_input("Enter object to pick up, or press return for default:")
    if len(goalObject) > 0:
        editedProblemFile = utils.setGoalObject(goalObject, editedProblemFile)
        
    myPatcher = PDDLPatcher(editedProblemFile)
    iterativePlanAuto(myPatcher, ORSetup, pddlDomainFile, editedProblemFile, viewer, envFile, planner)


if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:],"v")
    except getopt.GetoptError:
        print 'Use -v for viewer'
        sys.exit(2)

    viewer = False
    for opt,arg in opts:
        if opt == "-v":
            print "Setting viewer mode to True"
            viewer = True
            
    setupAndStart(pddlDomainFile, initialProblemFile, viewer, envFile, planner = 'ff')

    #pddlDomainFile = '../domains/dinnerTimeNoNegationCosts_dom.pddl'
    #pddlProblemFile = '../domains/dinnerTimeNoNegationCosts_prob.pddl'
    #plan, fdOutStr = runPlannerFD(pddlDomainFile, pddlProblemFile)
    #op = OutputParser("")
    #myPatcher.patchWithFDOutput(fdOutStr, 10)
    #propList = ["(not (smaller random_object11 random_object12))"]
    #myPatcher.patchWithProps(propList)
