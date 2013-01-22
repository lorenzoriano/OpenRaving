from PDDLPatcher import *
import sys, subprocess, time, os
import planning_primitives
from settings import *
import getopt
import StringIO
import glob
totalExecTime = 0

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
                     pollTime=10, planQuality=1, TimeLimit=350):
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


def updateInitFile(pddlProblemFile, iteration, plannerOutFname, \
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


def iterativePlanAuto(pddlDomainFile, pddlProblemFile, viewer, planner = "ff"):
    iteration = 0
    ex = planning_primitives.initOpenRave(viewer)
    cacheClearCount = 0
    while True:
        iteration += 1
        plannerOutFname = pddlProblemFile+ ".out"
        planCount = 0
        strPlanFileH, plannerOutStr, planCount = runPlanner(pddlDomainFile, pddlProblemFile, plannerOutFname, planner)
        prevPDDLFile = pddlProblemFile
        pddlProblemFile = initialProblemFile.replace(".pddl", repr(iteration) + ".pddl")
        
        reinterpret = "y"
        if strPlanFileH ==-1:
            reinterpret = raw_input("Planner failed. Reinterpret (y/n)?")
            if reinterpret != "y":
                print "Quitting"
                sys.exit(-1)
            myPatcher.forgetLearnedFactsAbout("obstructs")
            myPatcher.writeCurrentInitState(pddlProblemFile)
            planning_primitives.clearGPCache(ex)
            cacheClearCount += 1
            continue

        try:
            planning_primitives.test(strPlanFileH, ex)
            print "Success. Quitting."
            print "Total planning time: {0}".format(totalExecTime)
            print "Replan count: "+ repr(iteration)
            print "Cache clearing count: "+ repr(cacheClearCount)
            obsCount = tryIO(prevPDDLFile, "read").count("obstructs")
            print "Number of obstructions handled: "+ repr(obsCount)
            sys.exit(0)
        except planning_primitives.ExecutingException, e:
            errorStr = e.pddl_error_info
    
        if errorStr == "":
            print "Failure: Error in simulation. Try increasing number of samples."
            sys.exit(-1)

        print "Got facts:"
        print errorStr
        if viewer:
            #raw_input("Press return to continue")
            time.sleep(0.5)
        
            
        updateInitFile(pddlProblemFile, iteration, plannerOutFname, \
                       plannerOutStr, errorStr, planCount, planner)
        print

def main(argv):
    iteration = 0
    pddlProblemFile = initialProblemFile

    try:
        opts, args = getopt.getopt(argv[1:],"v")
    except getopt.GetoptError:
        print 'Use -v for viewer'
        sys.exit(2)

    viewer = False
    for opt,arg in opts:
        if opt == "-v":
            print "Setting viewer mode to True"
            viewer = True

            
    iterativePlanAuto(pddlDomainFile, initialProblemFile, viewer, planner="fd")


if __name__ == "__main__":
    myPatcher = PDDLPatcher(initialProblemFile)
    main(sys.argv)
    pddlDomainFile = '../domains/dinnerTimeNoNegationCosts_dom.pddl'
    #pddlProblemFile = '../domains/dinnerTimeNoNegationCosts_prob.pddl'
    #plan, fdOutStr = runPlannerFD(pddlDomainFile, pddlProblemFile)
    #op = OutputParser("")
    #myPatcher.patchWithFDOutput(fdOutStr, 10)
    #propList = ["(not (smaller random_object11 random_object12))"]
    #myPatcher.patchWithProps(propList)
