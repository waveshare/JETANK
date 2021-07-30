#!/usr/bin/env python3

import os
import numpy as np 

linkageLenA = 90
linkageLenB = 160

servoNumCtrl = [0,1]
servoDirection = [1,-1]

servoInputRange = 850
servoAngleRange = 180

servoInit = [None, 512, 512, 512, 512, 512]

nowPos = [None, 512, 512, 512, 512, 512]
nextPos = [None, 512, 512, 512, 512, 512]
speedBuffer = [None, 512, 512, 512, 512, 512]

xMax = 150
xMin = 90

yMax = 170
yMix = -170

if os.name == 'nt':
    import msvcrt
    print('nt')
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    #old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            #termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            pass
        return ch

# from scservo_sdk import *                    # Uses SCServo SDK library
# from jetbot.scservo_sdk import *  
from SCSCtrl.scservo_sdk import *

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_STS_GOAL_ACC          = 41
ADDR_STS_GOAL_POSITION     = 42
ADDR_STS_GOAL_SPEED        = 46
# ADDR_STS_PRESENT_POSITION  = 56
ADDR_SCS_PRESENT_POSITION  = 56

# Default setting
SCS1_ID                     = 1                 # SCServo#1 ID : 1
SCS2_ID                     = 2                 # SCServo#1 ID : 2
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyTHS1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 100               # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
SCS_MOVING_SPEED            = 0                 # SCServo moving speed
SCS_MOVING_ACC              = 0                 # SCServo moving acc
protocol_end                = 1                 # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


def syncCtrl(ID_List, Speed_List, Goal_List):
    positionList = []

    for i in range(0, len(ID_List)):
        try:
            scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, ID_List[i], ADDR_STS_GOAL_SPEED, Speed_List[i])
        except:
            time.sleep(0.1)
            scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, ID_List[i], ADDR_STS_GOAL_SPEED, Speed_List[i])

        positionBuffer = [SCS_LOBYTE(Goal_List[i]), SCS_HIBYTE(Goal_List[i])]
        positionList.append(positionBuffer)
    
    for i in range(0, len(ID_List)):
        scs_addparam_result = groupSyncWrite.addParam(ID_List[i], positionList[i])
    
    scs_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()


def infoSingleGet(SCID):
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCID, ADDR_SCS_PRESENT_POSITION)
    # if scs_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    # elif scs_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(scs_error))

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    # scs_present_speed = SCS_HIWORD(scs_present_position_speed)
    # print("[ID:%03d] PresPos:%03d PresSpd:%03d"%(SCID, scs_present_position, SCS_TOHOST(scs_present_speed, 15)))

    return scs_present_position


def portClose():
    portHandler.closePort()


def limitCheck(posInput, circlePos, circleLen, outline): #E
    circleRx = posInput[0]-circlePos[0]
    circleRy = posInput[1]-circlePos[1]
    realPosSquare = circleRx*circleRx+circleRy*circleRy
    shortRadiusSquare = np.square(circleLen[1]-circleLen[0])
    longRadiusSquare = np.square(circleLen[1]+circleLen[0])

    if realPosSquare >= shortRadiusSquare and realPosSquare <= longRadiusSquare:
        return posInput[0], posInput[1]

    else:
        lineK = (posInput[1]-circlePos[1])/(posInput[0]-circlePos[0])
        lineB = circlePos[1]-(lineK*circlePos[0])
        
        if realPosSquare < shortRadiusSquare:
            aX = 1 + lineK*lineK
            bX = 2*lineK*(lineB - circlePos[1]) - 2*circlePos[0]
            cX = circlePos[0]*circlePos[0] + (lineB - circlePos[1])*(lineB - circlePos[1]) - shortRadiusSquare

            resultX = bX*bX - 4*aX*cX
            x1 = (-bX + np.sqrt(resultX))/(2*aX)
            x2 = (-bX - np.sqrt(resultX))/(2*aX)

            y1 = lineK*x1 + lineB
            y2 = lineK*x2 + lineB

            if posInput[0] > circlePos[0]:
                if x1 > circlePos[0]:
                    xGenOut = x1+outline
                    yGenOut = y1
                else:
                    xGenOut = x2-outline
                    yGenOut = y2
            elif posInput[0] < circlePos[0]:
                if x1 < circlePos[0]:
                    xGenOut = x1-outline
                    yGenOut = y1
                else:
                    xGenOut = x2+outline
                    yGenOut = y2
            elif posInput[0] == circlePos[0]:
                if posInput[1] > circlePos[1]:
                    if y1 > circlePos[1]:
                        xGenOut = x1
                        yGenOut = y1+outline
                    else:
                        xGenOut = x2
                        yGenOut = y2-outline

            return xGenOut, yGenOut

        elif realPosSquare > longRadiusSquare:
            aX = 1 + lineK*lineK
            bX = 2*lineK*(lineB - circlePos[1]) - 2*circlePos[0]
            cX = circlePos[0]*circlePos[0] + (lineB - circlePos[1])*(lineB - circlePos[1]) - longRadiusSquare

            resultX = bX*bX - 4*aX*cX
            x1 = (-bX + np.sqrt(resultX))/(2*aX)
            x2 = (-bX - np.sqrt(resultX))/(2*aX)

            y1 = lineK*x1 + lineB
            y2 = lineK*x2 + lineB

            if posInput[0] > circlePos[0]:
                if x1 > circlePos[0]:
                    xGenOut = x1-outline
                    yGenOut = y1
                else:
                    xGenOut = x2+outline
                    yGenOut = y2
            elif posInput[0] < circlePos[0]:
                if x1 < circlePos[0]:
                    xGenOut = x1+outline
                    yGenOut = y1
                else:
                    xGenOut = x2-outline
                    yGenOut = y2
            elif posInput[0] == circlePos[0]:
                if posInput[1] > circlePos[1]:
                    if y1 > circlePos[1]:
                        xGenOut = x1
                        yGenOut = y1-outline
                    else:
                        xGenOut = x2
                        yGenOut = y2+outline

            return xGenOut, yGenOut


def planeLinkageReverse(linkageLen, linkageEnDe, servoNum, debugPos, goalPos): #E
    goalPos[0] = goalPos[0] + debugPos[0]
    goalPos[1] = goalPos[1] + debugPos[1]

    AngleEnD = np.arctan(linkageEnDe/linkageLen[1])*180/np.pi

    linkageLenREAL = np.sqrt(((linkageLen[1]*linkageLen[1])+(linkageEnDe*linkageEnDe)))

    goalPos[0],goalPos[1] = limitCheck(goalPos, debugPos, [linkageLen[0],linkageLenREAL], 0.00001)

    if goalPos[0] < 0:
        goalPos[0] = - goalPos[0]
        mGenOut = linkageLenREAL*linkageLenREAL-linkageLen[0]*linkageLen[0]-goalPos[0]*goalPos[0]-goalPos[1]*goalPos[1]
        nGenOut = mGenOut/(2*linkageLen[0])

        angleGenA = np.arctan(goalPos[1]/goalPos[0])+np.arcsin(nGenOut/np.sqrt(goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))
        angleGenB = np.arcsin((goalPos[1]-linkageLen[0]*np.cos(angleGenA))/linkageLenREAL)-angleGenA

        angleGenA = 90 - angleGenA*180/np.pi
        angleGenB = angleGenB*180/np.pi

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        linkagePointC = np.arcsin(goalPos[0]/goalPos[1])*180/np.pi*servoDirection[servoNumCtrl[0]]

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]

    elif goalPos[0] == 0:
        angleGenA = np.arccos((linkageLen[0]*linkageLen[0]+goalPos[1]*goalPos[1]-linkageLenREAL*linkageLenREAL)/(2*linkageLen[0]*goalPos[1]))
        cGenOut = np.tan(angleGenA)*linkageLen[0]
        dGenOut = goalPos[1]-(linkageLen[0]/np.cos(angleGenA))
        angleGenB = np.arccos((cGenOut*cGenOut+linkageLenREAL*linkageLenREAL-dGenOut*dGenOut)/(2*cGenOut*linkageLenREAL))

        angleGenA = - angleGenA*180/np.pi + 90
        angleGenB = - angleGenB*180/np.pi

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        linkagePointC = angleGenB + 90 - angleGenA

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]

    elif goalPos[0] > 0:
        sqrtGenOut = np.sqrt(goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1])
        nGenOut = (linkageLen[0]*linkageLen[0]+goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]-linkageLenREAL*linkageLenREAL)/(2*linkageLen[0]*sqrtGenOut)
        angleA = np.arccos(nGenOut)*180/np.pi

        AB = goalPos[1]/goalPos[0]

        angleB = np.arctan(AB)*180/np.pi
        angleGenA = angleB - angleA

        mGenOut = (linkageLen[0]*linkageLen[0]+linkageLenREAL*linkageLenREAL-goalPos[0]*goalPos[0]-goalPos[1]*goalPos[1])/(2*linkageLen[0]*linkageLenREAL)
        angleGenB = np.arccos(mGenOut)*180/np.pi - 90

        linkageLenC = np.sqrt((goalPos[0]*goalPos[0]+goalPos[1]*goalPos[1]))

        # linkagePointC = np.arcsin(goalPos[1]/goalPos[0])*180/np.pi*servoDirection[servoNumCtrl[0]]
        linkagePointC = 0

        anglePosC = angleGenB + angleGenA

        return [angleGenA*servoDirection[servoNumCtrl[0]], (angleGenB+AngleEnD)*servoDirection[servoNumCtrl[1]], linkageLenC, linkagePointC, anglePosC]


def servoAngleCtrl(ServoNum, AngleInput, DirectionDebug, SpeedInput):
    offsetGenOut = servoInit[ServoNum] + int((servoInputRange/servoAngleRange)*AngleInput*DirectionDebug)
    syncCtrl([ServoNum], [SpeedInput], [offsetGenOut])
    return offsetGenOut


def returnOffset(ServoNum, AngleInput, DirectionDebug):
    offsetGenOut = servoInit[ServoNum] + int((servoInputRange/servoAngleRange)*AngleInput*DirectionDebug)
    return offsetGenOut


def speedGenOut(angleList, timeInput):
    nowPos[2] = infoSingleGet(2)
    nowPos[3] = infoSingleGet(3)

    speedBuffer[2] = int(abs(nowPos[2] - returnOffset(2, angleList[0]+90, 1))*timeInput)
    speedBuffer[3] = int(abs(nowPos[3] - returnOffset(3, angleList[1], -1))*timeInput)

    print(speedBuffer[2], speedBuffer[3])


def xyInput(xInput, yInput):
    angGenOut = planeLinkageReverse([linkageLenA, linkageLenB], 0, servoNumCtrl, [0,0], [xInput, -yInput])
    servoAngleCtrl(2, angGenOut[0]+90, 1, 300)
    servoAngleCtrl(3, angGenOut[1], -1, 300)

    return [angGenOut[0], angGenOut[1]]


def nowPosUpdate(servoNumInput):
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, servoNumInput, ADDR_SCS_PRESENT_POSITION)
    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    
    nowPos[servoNumInput] = scs_present_position
    # print(scs_present_position)
    return scs_present_position


def speedGenOut(servoNum, dTime):
    dPos = abs(nextPos[servoNum] - nowPosUpdate(servoNum))
    try:
        speedBuffer[servoNum] = int(round(dPos/dTime,0))
    except:
        speedBuffer[servoNum] = 0

    return speedBuffer[servoNum]


def xyInputSmooth(xInput, yInput, dt):
    angGenOut = planeLinkageReverse([linkageLenA, linkageLenB], 0, servoNumCtrl, [0,0], [xInput, -yInput])

    nextPos[2] = returnOffset(2, angGenOut[0]+90, 1)
    nextPos[3] = returnOffset(3, angGenOut[1], -1)

    servoAngleCtrl(2, angGenOut[0]+90, 1, speedGenOut(2, dt))
    servoAngleCtrl(3, angGenOut[1], -1, speedGenOut(3, dt))

    return [angGenOut[0], angGenOut[1]]


def servoStop(servoNum):
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, servoNum, ADDR_SCS_PRESENT_POSITION)
    if scs_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print(packetHandler.getRxPacketError(scs_error))

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    scs_present_speed = SCS_HIWORD(scs_present_position_speed)
    # print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
    #       % (SCS_ID, scs_goal_position[index], scs_present_position, SCS_TOHOST(scs_present_speed, 15)))
    syncCtrl([servoNum], [0], [scs_present_position])


def stopServo(servoNumInput):
    try:
        servoStop(servoNumInput)
    except:
        time.sleep(0.1)
        servoStop(servoNumInput)


if __name__ == "__main__":
    xyInputSmooth(150, 170, 2)
    time.sleep(2)
    xyInputSmooth(150, -170, 2)
    xyInputSmooth(150, 50, 2)
    time.sleep(2)
    xyInputSmooth(150, 0, 2)


    # timeChange = 1.5
    # while 1:
    #     xyInputSmooth(100, 0, timeChange)
    #     print(nextPos)
    #     time.sleep(timeChange)

    #     xyInputSmooth(200, 0, timeChange)
    #     print(nextPos)
    #     time.sleep(timeChange)
    # xyInputSmooth(100, 0)


    # timeChange = 0.1
    # timeChangeOffset = 0.03
    # while 1:
    #     for i in range(-50, 50):
    #         xyInputSmooth(150+i, 0, timeChange)
    #         time.sleep(timeChange - timeChangeOffset)
    #     for i in range(50, -50, -1):
    #         xyInputSmooth(150+i, 0, timeChange)
    #         time.sleep(timeChange - timeChangeOffset)
    # xyInputSmooth(100, 0)

    # timeChange = 0.1
    # while 1:
    #     for i in range(-50, 50):
    #         xyInput(150+i, 0)
    #         time.sleep(timeChange)
    #     for i in range(50, -50, -1):
    #         xyInput(150+i, 0)
    #         time.sleep(timeChange)
    # xyInput(100, 0)

    # print(xyInput(240, 0))
    # a = planeLinkageReverse([linkageLenA, linkageLenB], 0, servoNumCtrl, [0,0], [150, 40])

    # servoAngleCtrl(2, 30, 1, 150)
    # servoAngleCtrl(1, 0, 1, 250)
    # time.sleep(1)
    # servoStop(2)
    '''
    ID_input     = [1, 2]
    Speed_input  = [500, 500]
    Goal_input_1 = [512, 512]
    Goal_input_2 = [1000, 1000]

    # syncCtrl(ID_input, Speed_input, Goal_input_2)

    while 1:
        # print(2)
        syncCtrl(ID_input, Speed_input, Goal_input_2)
        time.sleep(1)
        # print(1)
        syncCtrl(ID_input, Speed_input, Goal_input_1)
        time.sleep(1)

    time.sleep(1)
    print(infoSingleGet(1))

    # Close port
    portHandler.closePort()
    '''
