import collections, util, math, random

############################################################
# Problem 3.1.1

flag = False

def computeQ(mdp, V, state, action):
    """
    Return Q(state, action) based on V(state).  Use the properties of the
    provided MDP to access the discount, transition probabilities, etc.
    In particular, MDP.succAndProbReward() will be useful (see util.py for
    documentation).  Note that |V| is a dictionary.  
    """
    # BEGIN_YOUR_CODE (around 2 lines of code expected)
    total = 0
    discount = mdp.discount()
    for (succ, prob, reward) in mdp.succAndProbReward(state, action):
        total += prob*(reward + (discount * V[succ]))
    return total
    # END_YOUR_CODE

############################################################
# Problem 3.1.2

def policyEvaluation(mdp, V, pi, epsilon=0.001):
    """
    Return the value of the policy |pi| up to error tolerance |epsilon|.
    Initialize the computation with |V|.  Note that |V| and |pi| are
    dictionaries.
    """
    # BEGIN_YOUR_CODE (around 12 lines of code expected)
    mdp.computeStates()	
    states = mdp.states
    for state in states:
        V[state] = 0

    closeEnough = False
    while not closeEnough: 
        closeEnough = True
        newV = {}
        for state in states:
            newV[state] = computeQ(mdp, V, state, pi[state])
            if abs(newV[state] - V[state]) >= epsilon:
                closeEnough = False
        if closeEnough:
            return newV
        V = newV

    return V
    # END_YOUR_CODE

############################################################
# Problem 3.1.3

def computeOptimalPolicy(mdp, V):
    """
    Return the optimal policy based on V(state).
    You might find it handy to call computeQ().  Note that |V| is a
    dictionary.
    """
    # BEGIN_YOUR_CODE (around 4 lines of code expected)
    
    pi = {}
    for state in V:
        Qvals = []
        maxQ = float("-inf")
        bestAction = None
        for action in mdp.actions(state):
            q = computeQ(mdp, V, state, action)
            if q > maxQ:
                maxQ = q
                bestAction = action
        pi[state] = bestAction
    return pi
    # END_YOUR_CODE

############################################################
# Problem 3.1.4

class PolicyIteration(util.MDPAlgorithm):
    def solve(self, mdp, epsilon=0.001):
        mdp.computeStates()
        # compute |V| and |pi|, which should both be dicts
        # BEGIN_YOUR_CODE (around 11 lines of code expected)
        V = {}
        pi = {}
        for state in mdp.states:
            V[state] = 0
            pi[state] = None
        itsAMatch = False
        while not itsAMatch:
            itsAMatch = True
            newPi = computeOptimalPolicy(mdp, V)
            V = policyEvaluation(mdp, V, newPi, epsilon)
            for state in newPi:
                if pi[state] != newPi[state]:
                    itsAMatch = False
            pi = newPi

        # END_YOUR_CODE
        self.pi = pi
        self.V = V

############################################################
# Problem 3.1.5

class ValueIteration(util.MDPAlgorithm):
    def solve(self, mdp, epsilon=0.001):
        mdp.computeStates()
        # BEGIN_YOUR_CODE (around 13 lines of code expected)
        V = {}
        pi = {}
        for state in mdp.states:
            V[state] = 0
            pi[state] = None
        itsAMatch = False
        while not itsAMatch:
            itsAMatch = True
            pi = computeOptimalPolicy(mdp, V)
            newV = {}
            for state in V:
                newV[state] = computeQ(mdp, V, state, pi[state])
                if abs(V[state] - newV[state]) >= epsilon:
                    itsAMatch = False
            V = newV
    
        # END_YOUR_CODE
        self.pi = pi
        self.V = V

############################################################
# Problem 3.1.6

# If you decide 1f is true, prove it in writeup.pdf and put "return None" for
# the code blocks below.  If you decide that 1f is false, construct a
# counterexample by filling out this class and returning an alpha value in
# counterexampleAlpha().
class CounterexampleMDP(util.MDP):
    def __init__(self):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        return
        # END_YOUR_CODE

    def startState(self):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        return 0
        # END_YOUR_CODE

    # Return set of actions possible from |state|.
    def actions(self, state):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        return ['a']
        # END_YOUR_CODE

    # Return a list of (newState, prob, reward) tuples corresponding to edges
    # coming out of |state|.
    def succAndProbReward(self, state, action):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        if state is 0:
            return [(1, .2, 50), (2, .8, 0)]
        return []
        # END_YOUR_CODE

    def discount(self):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        return 1
        # END_YOUR_CODE

def counterexampleAlpha():
    # BEGIN_YOUR_CODE (around 1 line of code expected)
    return .2
    # END_YOUR_CODE

counterexampleAlpha()








############################################################
# Problem 3.2.1

class BlackjackMDP(util.MDP):
    def __init__(self, cardValues, multiplicity, threshold, peekCost):
        """
        cardValues: array of card values for each card type
        multiplicity: number of each card type
        threshold: maximum total before going bust
        peekCost: how much it costs to peek at the next card
        """
        self.cardValues = cardValues
        self.multiplicity = multiplicity
        self.threshold = threshold
        self.peekCost = peekCost

    # Return the start state.
    # Look at this function to learn about the state representation.
    # The first element of the tuple is the sum of the cards in the player's
    # hand.  The second element is the next card, if the player peeked in the
    # last action.  If they didn't peek, this will be None.  The final element
    # is the current deck.
    def startState(self):
        return (0, None, (self.multiplicity,) * len(self.cardValues))  # total, next card (if any), multiplicity for each card

    # Return set of actions possible from |state|.
    def actions(self, state):
        return ['Take', 'Peek', 'Quit']

    # Return a list of (newState, prob, reward) tuples corresponding to edges
    # coming out of |state|.  Indicate a terminal state (after quitting or
    # busting) by setting the deck to (0,).
    def succAndProbReward(self, state, action):
        # BEGIN_YOUR_CODE (around 50 lines of code expected)

        def removeCard(card):
            newDeck = list(state[2])
            newDeck[card] = newDeck[card]-1
            return tuple(newDeck)

        if  sum(state[2]) is 0:
            return []

        if action is 'Quit':
            return [((state[0], None, (0,)), 1, state[0])]

        if action is 'Peek':
            if state[1] is not None:
                return []
            possibleMoves = []
            for newCard in xrange(len(state[2])):
                possibleMoves.append(((state[0], newCard, state[2]), 
                    (state[2][newCard])/float(sum(state[2])), -1*self.peekCost))
            return possibleMoves 

        if action is 'Take':
            if state[1] is not None:
                return [((state[0]+state[1], None, removeCard(state[1])), 1, 0)]
            possibleMoves = []
            for newCard in xrange(len(state[2])):
                newCardValue = self.cardValues[newCard]
                prob = (state[2][newCard])/float(sum(state[2]))
                if prob == 0:
                    continue
                if state[0] + newCardValue > self.threshold:
                    possibleMoves.append(((state[0]+newCardValue, None, (0,)), 
                        prob, 0))
                else:
                    possibleMoves.append(((state[0]+newCardValue, None, 
                        removeCard(newCard)), prob, 0))
            if sum(state[2]) is 1:
                return [((possibleMoves[0][0][0], None, 
                        possibleMoves[0][0][2]), 1, possibleMoves[0][0][0])]
            return possibleMoves 

        

        # END_YOUR_CODE

    def discount(self):
        return 1

############################################################
# Problem 3.2.2

def peekingMDP():
    """
    Return an instance of BlackjackMDP where peeking is the optimal action at
    least 10% of the time.
    """
    # BEGIN_YOUR_CODE (around 2 lines of code expected)
    return BlackjackMDP([2, 15, 16], 3, 20, 1)
    # END_YOUR_CODE

