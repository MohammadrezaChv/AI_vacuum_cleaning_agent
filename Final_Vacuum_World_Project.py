from agents import *
from notebook import psource
from search import *
import random

psource(Agent)

class cleaner(Agent):
    location = [0,1]
    direction = Direction("up")


def program():
    '''Returns an action based on it's percepts'''
    return RandomAgentProgram(['right', 'left', 'up' , 'down' , 'Suck', 'NoOp'])


class XYEnvironments(Environment):
    """This class is for environments on a 2D plane, with locations
    labelled by (x, y) points, either discrete or continuous.

    Agents perceive things within a radius. Each agent in the
    environment has a .location slot which should be a location such
    as (0, 1), and a .holding slot, which should be a list of things
    that are held."""

    def __init__(self, width=10, height=10):
        super().__init__()

        self.width = width
        self.height = height
        self.observers = []
        # Sets iteration start and end (no walls).
        self.x_start, self.y_start = (0, 0)
        self.x_end, self.y_end = (self.width, self.height)

    perceptible_distance = 1

    def things_near(self, location, radius=None):
        """Return all things within radius of location."""
        if radius is None:
            radius = self.perceptible_distance
        radius2 = radius * radius
        return [(thing, radius2 - distance_squared(location, thing.location))
                for thing in self.things if distance_squared(
                location, thing.location) <= radius2]

    def percept(self, agent):
        """By default, agent perceives things within a default radius."""

        return self.things_near(agent.location)

    def execute_action(self, agent, action):
        agent.bump = False
        if action == 'TurnRight':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction += Direction.R
        elif action == 'TurnLeft':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction += Direction.L
        elif action == 'Forward':
            print('{} decided to move {}wards at location: {}'.format(str(agent)[1:-1], agent.direction.direction,
                                                                      agent.location))
            agent.bump = self.move_to(agent, agent.direction.move_forward(agent.location))
        elif action == 'left':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction = Direction("left")
            agent.bump = self.move_to(agent, agent.direction.move_forward(agent.location))
        elif action == 'right':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction = Direction("right")
            agent.bump = self.move_to(agent, agent.direction.move_forward(agent.location))
        elif action == 'up':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction = Direction("up")
            agent.bump = self.move_to(agent, agent.direction.move_forward(agent.location))
        elif action == 'down':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            agent.direction = Direction("down")
            agent.bump = self.move_to(agent, agent.direction.move_forward(agent.location))
        elif action == 'Grab':
            things = [thing for thing in self.list_things_at(agent.location) if agent.can_grab(thing)]
            if things:
                agent.holding.append(things[0])
                print("Grabbing ", things[0].__class__.__name__)
                self.delete_thing(things[0])
        elif action == 'Release':
            if agent.holding:
                dropped = agent.holding.pop()
                print("Dropping ", dropped.__class__.__name__)
                self.add_thing(dropped, location=agent.location)

    def default_location(self, thing):
        location = self.random_location_inbounds()
        while self.some_things_at(location, Obstacle):
            # we will find a random location with no obstacles
            location = self.random_location_inbounds()
        return location

    def move_to(self, thing, destination):
        """Move a thing to a new location. Returns True on success or False if there is an Obstacle.
        If thing is holding anything, they move with him."""
        thing.bump = self.some_things_at(destination, Obstacle)
        if not thing.bump:
            thing.location = destination
            for o in self.observers:
                o.thing_moved(thing)
            for t in thing.holding:
                self.delete_thing(t)
                self.add_thing(t, destination)
                t.location = destination
        return thing.bump

    def add_thing(self, thing, location=None, exclude_duplicate_class_items=False):
        """Add things to the world. If (exclude_duplicate_class_items) then the item won't be
        added if the location has at least one item of the same class."""
        if location is None:
            super().add_thing(thing)
        elif self.is_inbounds(location):
            if (exclude_duplicate_class_items and
                    any(isinstance(t, thing.__class__) for t in self.list_things_at(location))):
                return
            super().add_thing(thing, location)

    def is_inbounds(self, location):
        """Checks to make sure that the location is inbounds (within walls if we have walls)"""
        x, y = location
        return not (x < self.x_start or x > self.x_end or y < self.y_start or y > self.y_end)

    def random_location_inbounds(self, exclude=None):
        """Returns a random location that is inbounds (within walls if we have walls)"""
        location = (random.randint(self.x_start, self.x_end),
                    random.randint(self.y_start, self.y_end))
        if exclude is not None:
            while location == exclude:
                location = (random.randint(self.x_start, self.x_end),
                            random.randint(self.y_start, self.y_end))
        return location

    def delete_thing(self, thing):
        """Deletes thing, and everything it is holding (if thing is an agent)"""
        if isinstance(thing, Agent):
            del thing.holding

        super().delete_thing(thing)
        for obs in self.observers:
            obs.thing_deleted(thing)

    def add_walls(self):
        """Put walls around the entire perimeter of the grid."""
        for x in range(self.width):
            self.add_thing(Wall(), (x, 0))
            self.add_thing(Wall(), (x, self.height - 1))
        for y in range(1, self.height - 1):
            self.add_thing(Wall(), (0, y))
            self.add_thing(Wall(), (self.width - 1, y))

        # Updates iteration start and end (with walls).
        self.x_start, self.y_start = (1, 1)
        self.x_end, self.y_end = (self.width - 1, self.height - 1)

    def add_observer(self, observer):
        """Adds an observer to the list of observers.
        An observer is typically an EnvGUI.

        Each observer is notified of changes in move_to and add_thing,
        by calling the observer's methods thing_moved(thing)
        and thing_added(thing, loc)."""
        self.observers.append(observer)

    def turn_heading(self, heading, inc):
        """Return the heading to the left (inc=+1) or right (inc=-1) of heading."""
        return turn_heading(heading, inc)



class VacuumEnvironments(XYEnvironments):
    """The environment of [Ex. 2.12]. Agent perceives dirty or clean,
    and bump (into obstacle) or not; 2D discrete world of unknown size;
    performance measure is 100 for each dirt cleaned, and -1 for
    each turn taken."""

    def __init__(self, width=10, height=10):
        super().__init__(width, height)
        self.add_walls()

    def thing_classes(self):
        return [Wall, Dirt, ReflexVacuumAgent, RandomVacuumAgent,
                TableDrivenVacuumAgent, ModelBasedVacuumAgent]

    def percept(self, agent):
        """The percept is a tuple of ('Dirty' or 'Clean', 'Bump' or 'None').
        Unlike the TrivialVacuumEnvironment, location is NOT perceived."""

        status = ('Dirty' if self.some_things_at(
            agent.location, Dirt) else 'Clean')
        bump = ('Bump' if agent.bump else 'None')
        return status, bump


    def execute_action(self, agent, action):
        agent.bump = False
        if action == 'Suck':
            dirt_list = self.list_things_at(agent.location, Dirt)
            if dirt_list != []:
                print('{} suck {} at location: {}'
                      .format(str(agent)[1:-1], str(dirt_list[0])[1:-1], agent.location))
                dirt = dirt_list[0]
                agent.performance += 10
                self.delete_thing(dirt)

        else:
            super().execute_action(agent, action)

        if action != 'NoOp' and action != 'Suck':
            agent.performance -= 1
        elif action == 'NoOp':
            print('{} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))


#.....................................................................................
#Advanced Agent that I developed ....
psource(VacuumEnvironments)

def Advanced_Agent_Program():
    def program(percept):
        global optimal
        if optimal != []:
            action = optimal[0]
            optimal.pop(0)
            return action
        else :
            return 'NoOp'
    return program


def Envirnment(x , y):
    status = ['clean','dirty']
    aray = []
    for i in range(1,x+1):
        for j in range(1,y+1):
            temp = random.choice(status)
            if temp == 'dirty':
                aray.append(tuple([i,j]))
    return aray

def check_size(x , y):
    if x < 6 and x > 0:
        if y < 6 and y > 0:
            return True

class Vaccum_Problem(Problem):

    def __init__(self, initial, goal=()):
        #Inherits class Problem.
        super().__init__(initial, goal)
        self.total_cost = 0

    def actions(self, state):
        # Returns the possible actions the agent can perform in a given state.
        global xprime
        global yprime
        actions = ['left', 'right', 'up', 'down', 'Suck']
        x, y = state[0]

        # If the vacuum is on the left edge of 5x5 grid, remove action move left.
        if x == 1:
            actions.remove('left')
        # If the vacuum is on the rught edge of 5x5 grid, remove action move right.
        if x == xprime:
            actions.remove('right')
        # If the vacuum is on the bottom edge of 5x5 grid, remove action move down.
        if y == 1:
            actions.remove('down')
        # If the vacuum is on the top edge of 5x5 grid, remove action move up.
        if y == yprime:
            actions.remove('up')
        # If the square is not dirty, remove action suck.
        if (x, y) not in state[1]:
            actions.remove('Suck')

        return actions

    def result(self, state, action):
        # Return the state that results from executing an action in original state.

        new_state = list(state)
        x, y = state[0]

        # If the vacuum moves left, change the (x, y) location by decrementing x coordinate.
        if action == 'left':
            x -= 1
        # If the vacuum moves right, change the (x, y) location by incrementing x coordinate.
        elif action == 'right':
            x += 1
        # If the vacuum moves down, change the (x, y) location by decrementing y coordinate.
        elif action == 'down':
            y -= 1
        # If the vacuum moves up, change the (x, y) location by incrementing y coordinate.
        elif action == 'up':
            y += 1
        # If the vacuum performs suck, change state of s1. Remove current location (x, y)
        # from array of dirty squares.
        else:
            new_state[1] = list(state[1])
            new_state[1].remove((x, y))
            new_state[1] = tuple(new_state[1])

        # Set new state after executing actions.
        new_state[0] = list(state[0])
        new_state[0] = [x, y]
        new_state[0] = tuple(new_state[0])

        return tuple(new_state)

    def goal_test(self, state):
        #Return True if the state is a goal.

        return state[1] == self.goal

    def path_cost(self, c, state1, action, state2):
        # Return the cost of a solution path that arrives at state2 from state1 via action,
        # assuming cost c to get up to state1.

        self.total_cost = 1 + c + 2 * len(state2[1])
        return self.total_cost

    def remaining_dirty(self, count, state):
        # Return cost of remaining dirty spaces.

        # Calculate the cost of remaining dirty spaces.
        cost = 0
        for dirty in state[1]:
            countX, countY = count
            x, y = dirty

            if abs(y - countY) < 0 and abs(x - countX) < 0 and count != dirty:
                cost += 1

        return cost

    def clean_next(self, state):
        # Return the next dirty square to be cleaned by finding the closest dirty square.

        # Choose the next dirty square by calculating the distance to the nearest dirty square.
        closest = [[0, 0], 5000]
        for dirty in state[1]:
            dirty_distance = distance(state[0], dirty)

            if dirty_distance < closest[1]:
                closest = [dirty, dirty_distance]

        return closest

    def h1(self, node):
        # Return the heuristic value for a given state using h1.

        # Get the location and distance of the next dirty square to be cleaned.
        nearest_dirty, dirty_distance = self.clean_next(node.state)

        # Calculate the cost of h(1).
        return self.remaining_dirty(tuple(nearest_dirty), node.state) - dirty_distance

print('Maximum size of X and Y is 5 .')
x = int(input('Enter x : '))
y = int(input('Enter y : '))
xprime , yprime = x , y

psource(VacuumEnvironments)
if check_size(x,y):
    goal = tuple(Envirnment(x,y))
    start = (random.randint(1 , x) , random.randint(1 , y))
    print("Start location : " , start)
    print("Locations of Dirt at Envirnmet : " , goal)
    problem = Vaccum_Problem((start,((goal))))
    print("__________________________________________________________________________________________________________")
    print("Run Advanced Agent On The Envirnment (Tha Agent that I developed )")
    print("H(n): ")
    # Print the optimal action sequence to solve problem using A* search using heuristic funciton h.
    optimal = astar_search(problem, problem.h1, True).solution()
    print("Optimal path:", optimal)
    count = len(optimal)
    Env_2D = VacuumEnvironments(x + 2, y + 2)
    Advanced_Vacuum_Agent = cleaner(Advanced_Agent_Program())
    Env_2D.add_thing(Advanced_Vacuum_Agent, start)
    for d in goal:
        dirt = Dirt()
        Env_2D.add_thing(dirt, d)
        del(dirt)
    Env_2D.run(count)
    print('Percept Of Agent At Last Location ', Env_2D.percept(Advanced_Vacuum_Agent))
    print('Performance Score of Advanced Vacuum Agent : *',Advanced_Vacuum_Agent.performance,'*')
    # Print the optimal path's cost.
    #print("Optimal path cost:", problem.total_cost)

    print("__________________________________________________________________________________________________________")
    print("Run Random Agent On The Envirnment (Tha Agent Of The Book  )")
    Env_2Dprime = VacuumEnvironments(x + 2, y + 2)
    Random_Vacuum_Agent = cleaner(program())
    Env_2Dprime.add_thing(Random_Vacuum_Agent, start)
    for d in goal:
        dirt = Dirt()
        Env_2Dprime.add_thing(dirt, d)
        del(dirt)
    Env_2Dprime.run(count)
    #If Want to trace your Agent add this command "TraceAgent(Random_Vacuum_Agent)"
    print('Percept Of Agent At Last Location ', Env_2Dprime.percept(Random_Vacuum_Agent))
    print('Performance Score of Random Vacuum Agent : *',Random_Vacuum_Agent.performance , '*')

else :
    print('Wrong amout of x or y .... ')
    print('Try again ...')



