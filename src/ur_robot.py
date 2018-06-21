#!/usr/bin/env python

import sys
import rospy
import numpy as np
import urx


class urMotion:
    def __init__(self):

        # Way Points
        # self.init_wpr = (0.484, -0.0887, 0.178, 2.29, -2.17, -0.0778)
        self.init_wpr = (0.07173, 0.4035, 0.2231, 3.14, 0.0, 0.0)
        self.positions = {'position_1' : (),\
                          'position_2' : (),\
                          'position_3' : (),\
                          'position_4' : (),\
                          'position_5' : (),\
                          'position_6' : (),\
                          'position_7' : (),\
                          'position_8' : (),\
                          'position_9' : ()}

        # UR Setting & Move to Initial Point
        self.ur5 = urx.Robot("192.168.1.12")
        self.ur5.set_tcp((0.0, 0.0, 0.242, 0.0, 0.0, 0.0))
        self.lin_accel = 0.3
        self.lin_vel = 0.3
        self.x_offset = 0.15
        self.y_offset = 0.09
        self.fixed_z = 0.2231
        self.fixed_roll = 3.14
        self.fixed_pitch = 0.0
        self.fixed_yow = 0.0

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            go = raw_input("\n\n UR will move to the center of the table. Is it all clear? [y/n] : ") 
            if go == 'y':
                break
            else:
                continue
            rate.sleep()

        self.ur5.movel(self.init_wpr,self.lin_accel,self.lin_vel)
                
    def __del__(self):
        self.ur5.close()
        print 'ur-robot closed'

    def homing(self):
        self.ur5.movel(self.init_wpr,self.lin_accel,self.lin_vel)

    def pos_maker(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            current_pos = self.ur5.getl()
            print 'current position is ' + str(current_pos)
            start = raw_input("\n\n Start moving from here? [y/n] : ")
            if start == 'y':
                print start + '\n'
                break
            elif start == 'n':
                print start + '\n'
                print 'quit'
                return False
            else:
                print 'choose y or n'
                continue
            rate.sleep()
        
        current_pos = self.ur5.getl()

        position_1 = (current_pos[0] - self.x_offset, current_pos[1] + self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_2 = (current_pos[0], current_pos[1] + self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_3 = (current_pos[0] + self.x_offset, current_pos[1] + self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_4 = (current_pos[0] - self.x_offset, current_pos[1], self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_5 = (current_pos[0], current_pos[1], self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_6 = (current_pos[0] + self.x_offset, current_pos[1], self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_7 = (current_pos[0] - self.x_offset, current_pos[1] - self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_8 = (current_pos[0], current_pos[1] - self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
        position_9 = (current_pos[0] + self.x_offset, current_pos[1] - self.y_offset, self.fixed_z, self.fixed_roll, self.fixed_pitch, self.fixed_yow)
   
        self.positions['position_1'] = position_1
        self.positions['position_2'] = position_2
        self.positions['position_3'] = position_3
        self.positions['position_4'] = position_4
        self.positions['position_5'] = position_5
        self.positions['position_6'] = position_6
        self.positions['position_7'] = position_7
        self.positions['position_8'] = position_8       
        self.positions['position_9'] = position_9
        
        for i in range(1, len(self.positions)+1):
            if self.positions['position_' + str(i)][1] > 0.560:
                rospy.logerr('UR will hit the Sawyer!! : %f', self.positions['position_' + str(i)][1])
                return False
            elif self.positions['position_' + str(i)][1] < 0.170:
                rospy.logerr('y value is too close to 0 : %f', self.positions['position_' + str(i)][1])
                return False

        return True

    def take_a_picture(self):
        rospy.set_param('/save_or_not', True)

    def move(self, num):
        position = self.positions['position_' + str(num)]        
        print 'going to ' + str(position) + '...\n'
        self.ur5.movel(position,self.lin_accel,self.lin_vel)



NAME = 'ur_robot'

def main():

    rospy.init_node(NAME)
    ur = urMotion();

    if ur.pos_maker() == True:
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            ur.move(1)
            ur.take_a_picture()
            ur.move(2)
            ur.take_a_picture()
            ur.move(3)
            ur.take_a_picture()
            ur.move(6)
            ur.take_a_picture()
            ur.move(5)
            ur.take_a_picture()
            ur.move(4)
            ur.take_a_picture()
            ur.move(7)
            ur.take_a_picture()
            ur.move(8)
            ur.take_a_picture()
            ur.move(9)
            ur.take_a_picture()

            start = raw_input("\n\n Ready to do next round? [y/n] : ")
            if start == 'y':
                print start + '\n'
            else:
                print start + '\n'
                print 'quit'
                break
            
            rate.sleep()
    del ur


    
if __name__ == '__main__':
    main()

