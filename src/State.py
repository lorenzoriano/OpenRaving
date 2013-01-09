import re, pdb

class State:
    def __init__(self):
        self.__trueSet = set();
        self.__falseSet = set();

    def addProposition(self, propStr):
        r = re.compile("\(not-", re.IGNORECASE)
        
        if r.match(propStr) == None:
            self.addTrue(propStr.strip())
        else:
            self.addFalse(r.sub("(", propStr.strip()))

    def addTrue(self, propStr):
        self.__trueSet.add(propStr)

    def addFalse(self, propStr):
        self.__falseSet.add(propStr)

    def getTrueProps(self):
        return self.__trueSet

    def getFalseProps(self):
        return self.__falseSet

    def getAllProps(self):
        return self.getTrueProps()|self.getFalseProps()

    def printState(self):
        print("False: ")
        if len(self.__falseSet)>0:
            l = list(self.__falseSet)
            l.sort()
            print "\n".join(l)
        print("True: ")
        if len(self.__trueSet)>0:
            l = list(self.__trueSet)
            l.sort()
            print "\n".join(l)
        
        print
        print
    
    def size(self):
        return len(self.__trueSet) + len(self.__falseSet)

    def patch(self, deltaState):
        for prop in deltaState.getFalseProps():
            self.__trueSet.discard(prop)

        for prop in deltaState.getTrueProps():
            self.__trueSet.add(prop)
            self.__falseSet.discard(prop)

    def removeTrueProp(self, prop):
        self.__trueSet.discard(prop)

    def removeFalseProp(self, prop):
        self.__falseSet.discard(prop)

    def makeCWAExplicit(self, propSet):
        for prop in propSet:
            if prop not in self.__trueSet:
                self.__falseSet.add(prop)
                

                
