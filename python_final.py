import sys, random, math, pygame
from pygame.locals import *
import time
from math import atan, cos, sin, atan2
from RRT_includes import *

# constants
XDIM = 720
YDIM = 500
WINSIZE = [XDIM, YDIM]
EPSILON = 5
NUMNODES = 20000
GOAL_RADIUS = 5

pygame.init()
fpsClock = pygame.time.Clock()

# initialize and prepare screen
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('Rapidly Exploring Random Tree')
white = 255, 255, 255
black = 20, 20, 40
red = 255, 0, 0
green = 0, 255, 0
blue = 0, 0, 255
cyan = 0, 255, 255
yellow = 255, 255, 0

# setup program variables
count = 0
rectObs = []


def angle(p1, p2, p3):
    m1 = (p2[1] - p1[1]) / (p2[0] - p1[0])
    m2 = (p3[1] - p2[1]) / (p3[0] - p2[0])
    k = (m1 - m2) / (1 + (m1 * m2))
    ang = atan(k)
    return ang


def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def collides(p):
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            # print ("collision with object: " + str(rect))
            return True
    return False


def get_random():
    return random.random() * XDIM, random.random() * YDIM


def get_random_clear():
    while True:
        p = get_random()
        noCollision = collides(p)
        if noCollision == False:
            return p


def init_obstacles():
    global rectObs
    rectObs = []
    rectObs.append(pygame.Rect((70, 20), (100, 100)))
    rectObs.append(pygame.Rect((200, 20), (100, 100)))
    rectObs.append(pygame.Rect((330, 20), (100, 100)))
    rectObs.append(pygame.Rect((460, 20), (100, 100)))
    rectObs.append(pygame.Rect((590, 20), (100, 100)))
    rectObs.append(pygame.Rect((490, 150), (200, 200)))
    rectObs.append(pygame.Rect((70, 150), (380, 200)))
    rectObs.append(pygame.Rect((70, 380), (100, 100)))
    rectObs.append(pygame.Rect((200, 380), (100, 100)))
    rectObs.append(pygame.Rect((330, 380), (100, 100)))
    rectObs.append(pygame.Rect((460, 380), (100, 100)))
    rectObs.append(pygame.Rect((590, 380), (100, 100)))

    for rect in rectObs:
        pygame.draw.rect(screen, yellow, rect)
    # print(len(rectObs))


def reset():
    global count
    screen.fill(black)
    init_obstacles()
    count = 0


def init_obstacles1():
    global rectObs
    rectObs = []
    rectObs.append(pygame.Rect((70, 20), (100, 100)))
    rectObs.append(pygame.Rect((200, 20), (100, 100)))
    rectObs.append(pygame.Rect((330, 20), (100, 100)))
    rectObs.append(pygame.Rect((460, 20), (100, 100)))
    rectObs.append(pygame.Rect((590, 20), (100, 100)))
    rectObs.append(pygame.Rect((490, 150), (200, 200)))
    rectObs.append(pygame.Rect((70, 150), (380, 200)))
    rectObs.append(pygame.Rect((70, 380), (100, 100)))
    rectObs.append(pygame.Rect((200, 380), (100, 100)))
    rectObs.append(pygame.Rect((330, 380), (100, 100)))
    rectObs.append(pygame.Rect((460, 380), (100, 100)))
    rectObs.append(pygame.Rect((590, 380), (100, 100)))
    rectObs.append(pygame.Rect((460, 250), (30, 30)))

    for rect in rectObs:
        pygame.draw.rect(screen, yellow, rect)
    # print(len(rectObs))


def reset1():
    global count
    screen.fill(black)
    init_obstacles1()
    count = 0


def main():
    global count

    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'
    itr = 0

    nodes = []
    backtrack = []
    back = []
    reset()

    while True:
        if currentState == 'init':
            # print('goal point not yet set')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            # traceback
            currNode = goalNode.parent
            while currNode.parent is not None:
                pygame.draw.line(screen, green, currNode.point, currNode.parent.point)
                backtrack.append(currNode.point)
                # back = [list(ele) for ele in backtrack]
                currNode = currNode.parent
                if currNode.parent is None:
                    if pygame.Rect((460, 250), (30, 30)) not in rectObs:
                        # time.sleep(5)
                        # rectObs.append(pygame.Rect((460, 250), (30, 30)))
                        # pygame.draw.rect(screen, yellow, pygame.Rect((460, 250), (30, 30)))
                        itr = 1
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count + 1
            if count < NUMNODES:
                foundNext = False
                while not foundNext:
                    rand = get_random_clear()
                    # print("random num = " + str(rand))
                    parentNode = nodes[0]

                    for p in nodes:  # find nearest vertex
                        if dist(p.point, rand) <= dist(parentNode.point,
                                                       rand):  # check to see if this vertex is closer than the previously selected closest
                            newPoint = step_from_to(p.point, rand)
                            if not collides(
                                    newPoint):  # check if a collision would occur with the newly selected vertex
                                parentNode = p  # the new point is not in collision, so update this new vertex as the best
                                foundNext = True

                newnode = step_from_to(parentNode.point, rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen, white, parentNode.point, newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes) - 1]

                # if count % 100 == 0:
                # print("node: " + str(count))
            else:
                print("Ran out of nodes... :(")
                return

        # handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                # print('mouse down')
                if currentState == 'init':
                    if not initPoseSet:
                        nodes = []
                        if not collides((40, 365)):
                            # print('initiale pose set: ' + str(e.pos))
                            # print(e.pos, "epos")
                            initialPoint = Node((40, 365), None)
                            nodes.append(initialPoint)  # Start in the center
                            initPoseSet = True
                            # pygame.draw.circle(screen, blue, initialPoint.point, GOAL_RADIUS)
                            pygame.draw.rect(screen, red, (initialPoint.point[0] - 3, initialPoint.point[1] - 3, 6, 6),
                                             0)
                    elif not goalPoseSet:
                        # print('goal pose set: ' + str(e.pos))
                        if collides((680, 130)) == False:
                            goalPoint = Node((680, 130), None)
                            goalPoseSet = True
                            # pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            pygame.draw.rect(screen, green, (goalPoint.point[0] - 3, goalPoint.point[1] - 3, 6, 6), 0)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        if itr == 1:
            break
        # time.sleep(2)
    # print(len(rectObs), "check")
    # print(backtrack, "back")
    bt_final = backtrack[::-1]
    # print(bt_final, "bt")

    angles = []
    for y in range(len(bt_final) - 2):
        z = angle(bt_final[y], bt_final[y + 1], bt_final[y + 2])
        angles.append(z)
    print("angles", angles)

    time.sleep(5)
    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset1()

    while True:
        if currentState == 'init':
            # print('goal point not yet set')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            # traceback
            currNode = goalNode.parent
            while currNode.parent is not None:
                pygame.draw.line(screen, green, currNode.point, currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count + 1
            if count < NUMNODES:
                foundNext = False
                while not foundNext:
                    rand = get_random_clear()
                    # print("random num = " + str(rand))
                    parentNode = nodes[0]

                    for p in nodes:  # find nearest vertex
                        if dist(p.point, rand) <= dist(parentNode.point,
                                                       rand):  # check to see if this vertex is closer than the previously selected closest
                            newPoint = step_from_to(p.point, rand)
                            if not collides(
                                    newPoint):  # check if a collision would occur with the newly selected vertex
                                parentNode = p  # the new point is not in collision, so update this new vertex as the best
                                foundNext = True

                newnode = step_from_to(parentNode.point, rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen, white, parentNode.point, newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes) - 1]

                # if count % 100 == 0:
                # print("node: " + str(count))
            else:
                print("Ran out of nodes... :(")
                return

        # handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                # print('mouse down')
                if currentState == 'init':
                    if not initPoseSet:
                        nodes = []
                        if not collides((460, 300)):
                            # print('initiale pose set: ' + str(e.pos))
                            # print(e.pos, "epos")

                            initialPoint = Node((460, 300), None)
                            nodes.append(initialPoint)  # Start in the center
                            initPoseSet = True
                            # pygame.draw.circle(screen, blue, initialPoint.point, GOAL_RADIUS)
                            pygame.draw.rect(screen, red,
                                             (initialPoint.point[0] - 3, initialPoint.point[1] - 3, 6, 6),
                                             0)
                    elif not goalPoseSet:
                        # print('goal pose set: ' + str(e.pos))
                        if collides((680, 130)) == False:
                            goalPoint = Node((680, 130), None)
                            goalPoseSet = True
                            # pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            pygame.draw.rect(screen, green, (goalPoint.point[0] - 3, goalPoint.point[1] - 3, 6, 6),
                                             0)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        # time.sleep(2)
        fpsClock.tick(10000)


# if python says run, then we should run
if __name__ == '__main__':
    main()

# angles [1.168103170433286, -0.8516113810848958, -0.1267082981823443, 1.0254496123384753, -1.2707392709947722,
# 1.368076990258736, -0.09916970053122368, 0.7273408999436561, -0.6064465201053787, -0.139596035538523,
# -0.958368675396469, 0.9727766365143868, -1.099145475624749, 1.2479157141955932, 0.01257777866354152,
# -1.5218740877759656, 0.7589241224650525, 0.03478073290276045, 0.7648919434270594, -1.1950019722671474,
# -0.655469740195676, 1.5277745563870428, -1.490156408539466, 0.10922739191708979, 1.3003740396948225,
# -0.15237352861815093, 0.405133551694528, -0.21766906800818553, 0.320439807102834, 0.7754751238256571,
# -1.4979986577212943, -0.8385741872608663, -0.12494456339548517, 0.6715979396169787, -0.10439427220345061,
# 0.3309317896951861, -0.008576694197277482, 0.07631813328942368, -0.16297490911923138, -1.0358857484466915,
# 0.17970290755230345, -0.1563848317128526, -0.0727103156150596, 0.9879394624428557, -0.9706581345755858,
# 1.0524540219571343, 0.21541730385678526, -0.7312255737775593, -0.408804994117254, -0.08722124621672064,
# 0.3770487804207059, 0.008015222366135387, 0.2751592648090455, -0.8225284293052864, 1.3735101581194735,
# -0.9993534690926373, 0.007540743119695413, 0.008766673553514522, -0.5830822597955971, 0.42846168665550677,
# 0.6171933871028722, 0.15850085948432582, -1.03422926889879, 0.5873740831702253, -0.5526061042221015,
# 0.8932607864769703, -0.7004628096402973, 0.5539946894125802, -0.38258439671570377, -0.21880261415363267,
# 0.258728686950552, -0.194316025299936, 0.7372967671265452, -0.6391365269928656, -0.08200040032177475,
# -0.06851925464200762, 0.25057297356640784, -0.0570709370631732, -0.09055404034281561, 0.653399589036376,
# -0.9178494478408934, 0.687236932599642, -0.4260276098825413, -0.0729543967791698, 0.22227648474915185,
# 0.9542465982591428, -1.4740151324291986, 0.40802758168358244, 0.16411146235932625, 0.6716447444025095,
# -0.9403727596478878, 0.16459115659896295, -0.3779763540020463, 1.0813472478466126, -1.0098471819684796,
# 0.48173179246439346, 0.2893315070226514, -0.6398707245787514, 0.09049458500854779, -0.21781400258463335,
# 1.0238613100314737, -0.4768168852553247, -0.7194668999682322, 0.9821351366545044, -1.0257719462903232,
# 0.5843124825038386, -0.3533961243080494, 1.0510825238503774, -0.7192991320143047, 0.3908942963662084,
# -0.5416699026051273, 0.06071127738471287, -0.6035713847694505, -1.3500448103396103, -1.5079181957507901,
# -0.2424783423702443, 0.49994723789442863, 0.31936741298345683, -0.8329314620211966, 1.461388492286652,
# 0.12649818426282855, 0.6860985029422956, 0.09593508666286898, -0.8161905503206993, 0.7600148409708126,
# -0.13121135843639323, 0.15719699130133746, 0.45441944386390953, -0.9468202174416692, 0.37314557939127785,
# -0.210316517271283, 0.4326546340283058, -0.16425022039170722, 0.16653943858568632, -0.12605359590348786,
# 0.14844610135270903, 0.17783478251806695, -0.3350697788815479, -0.06679368757551225, 0.4908974565330134,
# -0.9522998090444608, -0.8013197377979612, 0.7857109623807812, -0.02950256156810967, -0.10161207906787524,
# -0.2521023546925387, 0.5265131055467818, 0.1506028226725365, 0.9712149117204326, -0.4806124839694118,
# -1.199497539711153, 0.6407122585096571, -0.09667218263686224, 0.878820086867238, -1.0438087392924797,
# 0.8439192913754567, -0.04457449820218698, 0.7749298478067806, 1.2732499823349548, 0.6011416502400402,
# 0.3356903563337997, 0.35000282292543866, -1.373795025544622, 1.1150374981343, -0.1391389099486143,
# 1.3662098269660619, 1.0062326621933366, -0.4739575311030118, 0.012331819054591308, 0.6333167271991886,
# -0.057759968239718754, -1.0840601278415216, 0.19311540241984954, 0.6243344389879479, -0.2996029663674338,
# -0.3330592738085109, 0.9499735749921098, -1.09983369823904, 0.7133871080368329, 0.07702702066222754,
# -0.7311063067116982, -0.395654640674047, 0.8943291043704407, -0.30054435840624116, 0.1672169062098167,
# 0.7626362558705135, -1.20578003828719, 0.30916110610341746, 0.6945624135429292, -1.2545656740022255,
# -0.11443302379874519, -1.089156196944266, 0.9431945007416157, -0.8189089710829807, 1.5596063451700188,
# 0.1794486176906383, -0.2232371261362427, -0.24392781126341892, -0.4760250802190383, -0.21068537497042425,
# 1.4533188812149196, -1.0319008716525162, 0.09746980544789455, -1.2496205707719104, 0.8246034679433187,
# 0.3828245702569821, -0.060134718972645315, 1.257702431566994, -1.1070914696942111, 0.8935853633312608,
# -0.2649677982518993, -0.8277677385830751, 1.2020467129065235, 0.4315504115989613, -0.028239081871509114,
# -0.496274536090068, -1.2660690705983257, -1.3527132981941783, -1.557856567809996, 1.5581196881685344,
# -0.573890827358957, 0.8256786204141684, -0.0007397619728530353, 0.3056253901151239, -0.4624474081421653,
# -0.08258325121366017, -0.4523181946588066, 0.6321541064726732, 0.796881497614944, -0.6249261988507678,
# -0.321546101341337, 0.152169647335339, -0.8947682623357077, 1.1198733062894346, -0.5845949682049224,
# 0.530287525220472, 0.7253312079379453, -0.8774558115762442, 0.7943258124936268, 0.1026901349010208,
# -0.3564837335306893, -0.21854274885542394, 0.6489879873736963, -0.3859741938716845, -1.533485617184617,
# -0.8455767796479188, -1.3403358599365172, 0.2650403365963484]

# vel_list = [[5, 5], [1, -1], [3, 3], [-1, 1], [5, 5], [2, -2], [5, 5], [-1, 1], [5, 5], [-1, 1], [10, 10], [1, -1],
# [5, 5], [-1, 1], [5, 5], [-1, 1], [5, 5], [1, -1], [5, 5], [1, -1], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [1,
# -1], [5, 5], [-1, 1], [5, 5], [5, 5], [-1, 1], [5, 5], [1, -1], [5, 5], [5, 5], [5, 5], [-2, 2], [5, 5], [5, 5],
# [-2, 2], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [5, 5], [1, -1], [5, 5], [5, 5], [5, 5],
# [2, -2], [5, 5], [5, 5], [2, -2], [5, 5], [5, 5], [1, -1], [5, 5], [5, 5], [1, -1], [5, 5], [-1, 1], [5, 5], [5,
# 5], [1, -1], [5, 5], [1, -1], [5, 5], [5, 5], [-1, 1], [5, 5]]


try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    # res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    # if res==vrep.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    # else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
    # retrieve motor  handles
    errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_left_joint', vrep.simx_opmode_blocking)
    errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_right_joint', vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    v = [[5, 5], [1, -1], [3, 3], [-1, 1], [5, 5], [2, -2], [5, 5], [-1, 1], [5, 5], [-1, 1], [10, 10], [1, -1], [5, 5],
         [-1, 1], [5, 5], [-1, 1], [5, 5], [1, -1], [5, 5], [1, -1], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [1, -1],
         [5, 5], [-1, 1], [5, 5], [5, 5], [-1, 1], [5, 5], [1, -1], [5, 5], [5, 5], [5, 5], [-2, 2], [5, 5], [5, 5],
         [-2, 2], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [-1, 1], [5, 5], [5, 5], [5, 5], [1, -1], [5, 5], [5, 5],
         [5, 5], [2, -2], [5, 5], [5, 5], [2, -2], [5, 5], [5, 5], [1, -1], [5, 5], [5, 5], [1, -1], [5, 5], [-1, 1],
         [5, 5], [5, 5], [1, -1], [5, 5], [1, -1], [5, 5], [5, 5], [-1, 1], [5, 5]]
    path_speeds = v

    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        # print(type(k[0]))
        while (err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            # print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            # print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while (time < 1):
            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)


    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can
    # guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
