from State import *
from OutputParser import *

## parse pddl init state
## remove those that occur as false
## add those that occur as true

class InitFileMgr:
    def __init__(self, fname):
        self.pddlFName = fname
        self.initState = self.getInitStateFromString() # the state to be maintained
        self.patchSequence = [] # history of applied patches 
        self.patchedStateSequence = [] # history of init states

    def getPDDLStr(self):
        pddlStr = tryIO(self.pddlFName, "read")
        return pddlStr

    def writeFile(self, ofname, txt):
        tryIO(ofname, "write", txt)
            
    def pushCurrentInitToHistory(self):
        s=State()
        s.patch(self.initState)
        self.patchedStateSequence.append(s)
        
    def replaceInitState(self, newState):
        self.pushCurrentInitToHistory()
        self.patchSequence.append('replaced state')
        self.initState = State()
        self.initState.patch(newState)
     
     
    def getCurrentState(self):
        return self.initState

    def extractInitSection(self):
        pddlStr = self.getPDDLStr()

        initSection = ""

        for section in pddlStr.split("(:"):
            if section.strip().find("init") == 0:
                initSection = section.replace("init", "")
                break
        return initSection
    
    def getInitStateFromString(self):
        op = OutputParser("")        
        return op.getStateFromStr(self.extractInitSection())

    def patchInitState(self, newState):
        s = State()
        s.patch(self.initState)
        self.patchedStateSequence.append(s)

        self.initState.patch(newState)
        self.patchSequence.append(newState)
    
    def printInitState(self):
        print "Patch generation " + repr(len(self.patchSequence))
        self.initState.printState()

    def writeCurrentPDDL(self, ofname):
        print "Writing PDDL file " + ofname
        pddlStr = self.getPDDLStr()
        pddlOutStr = ""
        propList = list(self.initState.getTrueProps())
        propList.sort()
        
        pddlPart1 = pddlStr.split("(:init")[0]
        pddlPart2 = pddlStr.split("(:init")[1].strip().split("(:goal")[1]
        pddlOutStr = pddlPart1 + "\n\n(:init \n" + "\n".join(propList) + ")" +\
            "\n\n(:goal" + pddlPart2

        # for section in pddlStr.split("(:"):
        #     if section.strip().find("init") != 0:
        #         pddlOutStr += "(:" + section + "\n\n"
        #     else:
        #         pddlOutStr += "(:init \n" + \
        #             "\n".join(propList)
        #         pddlOutStr += "\n"

        self.writeFile(ofname, pddlOutStr)

    def rollbackTo(self, stateNum):
        s = State()
        s.patch(self.patchedStateSequence[stateNum])
        self.initState = s
        self.patchSequence.append("Rollback-"+repr(stateNum))
        self.patchedStateSequence.append(s)

    def purgeFactsWithSymbol(self, symbol):
        #In order to log the change, create a state
        # with facts that match symbol
        # remove from both true set and false set
        s = State()        
        s.patch(self.initState)
        self.patchSequence.append("Purge symbol: "+symbol)
        self.patchedStateSequence.append(s)
        
        for prop in s.getTrueProps():
            if symbol in prop:
                self.initState.removeTrueProp(prop)

        for prop in s.getFalseProps():
            if symbol in prop:
                self.initState.removeFalseProp()



if __name__ == "__main__":
    myPDDL = InitFileMgr("/tmp/test2.out")
    op = OutputParser("/tmp/test.out")
    pset = op.getPropSet()
    print "\n\nAll props:"
    print str(pset)
    myPDDL.printInitState()
    myPDDL.patchInitState(op.stateList[2], pset)
    print "After patch:"
    myPDDL.printInitState()
