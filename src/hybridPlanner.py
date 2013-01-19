from PDDLPatcher import *
import sys, subprocess, time, os
import planning_primitives
from settings import *
import getopt
import StringIO
totalExecTime = 0

def execCmd(cmd,  successStr,  dumpFile, pollTime = 2):
    ''' A generic utility for executing a command. 
    Dumpfile stores stdout and stderr'''
    initTime = time.time()
    print "Executing %s...   " %(cmd), 
    sys.stdout.flush()
    ## Using subprocess.call so that all the messages from the planner
    ## can be captured and parsed for the string "Solution found!"

    #dumpF = open(dumpFile,  "w")
    #p = subprocess.Popen([cmd], shell = True, stdout=dumpF, stderr=dumpF)
    startTime = time.time()
    p = subprocess.Popen([cmd], shell = True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    while p.poll == None:
        time.sleep(pollTime)
        if time.time() - startTime >10:
            ans = raw_input("Still running. Continue (c), Restart (r), Quit(q)? ")
            startTime = time.time()
            if ans.strip() == "q":
                os.kill(p.pid+1,  9)
                sys.exit(-1)
            elif ans.strip() == "c":
                continue
            elif ans.strip() == "r":
                os.kill(p.pid+1,  9)
                return execCmd(cmd, successStr, dumpFile)
            else:
                print "Unrecognized response. Continuing."

    #dumpF.close()
    #time.sleep(5)
    endTime = time.time()

    #dumpF = open(dumpFile, "r")
    #msg = dumpF.read()
    out, err = p.communicate()
    msg = out+err
    
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


def iterativePlanAuto(pddlDomainFile, pddlProblemFile, viewer):
    iteration = 0
    ex = planning_primitives.initOpenRave(viewer)
    while True:
        iteration += 1
        ffCmdLine = ff + " -o " + pddlDomainFile +" -f " + pddlProblemFile
        ffOutputFile = pddlProblemFile+ ".out"

        retVal = execCmd(ffCmdLine, "found legal plan",  ffOutputFile)
        if retVal ==-1:
                sys.exit(-1)
        
        tryIO(ffOutputFile, "write", retVal)
        ffOutStr = OutputParser(ffOutputFile).getFFPlan()

        print "Computed plan: \n" + ffOutStr
        #planFile = pddlProblemFile+".plan"
        #tryIO(planFile, "write", ffOutStr)
        #time.sleep(2)
        strPlanFile = StringIO.StringIO()
        strPlanFile.write(ffOutStr)
        strPlanFile.seek(0)
    
        try:
            planning_primitives.test(strPlanFile, ex)
            print "Success. Quitting."
            print "Total planning time: {0}".format(totalExecTime)
            sys.exit(0)
        except planning_primitives.ExecutingException, e:
            errorStr = e.pddl_error_info
    
        if errorStr == "":
            print "Failure: Error in simulation. Try increasing number of samples."
            sys.exit(-1)

        print "Got facts:"
        print errorStr
        if viewer:
            raw_input("Press return to continue")
#        else:
#            time.sleep(5)
    
        errorLines = errorStr.lower().split("\n")
    
        failedActionNumber = int(errorLines[0].strip("linenumber: "))
        propList = filter( lambda x: len(x)>0, errorLines[1:])
    
        ## If action i failed, need state just after action i-1
        ## kth element of state list from ffoutputfile is 
        ## the state before application of action k.
        myPatcher.patchWithFFOutput(ffOutputFile, failedActionNumber)
        myPatcher.patchWithProps(propList)
        print "State to re-plan from:"
        myPatcher.printInitState()
    
        pddlProblemFile = initialProblemFile.replace(".pddl", \
                                                         repr(iteration)+".pddl")
        myPatcher.writeCurrentInitState(pddlProblemFile)

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

            
    iterativePlanAuto(pddlDomainFile, initialProblemFile, viewer)


if __name__ == "__main__":
    myPatcher = PDDLPatcher(initialProblemFile)
    #myPatcher.printInitState()
    main(sys.argv)
