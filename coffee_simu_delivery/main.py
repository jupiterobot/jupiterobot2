#!/usr/bin/env python

import os
import sys
from PyQt5 import QtWidgets, uic
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('coffee_delivery.ui', self)  # load UI file
        
        print("waiting for command..")
        self.coffee_ready = False
        self.coffee_exist = [True, True, True] # coffee 1, 2, 3 exist
        self.coffee_hold = False 
        
        # button event
        self.startTaskButton.clicked.connect(self.start_task)
        self.BackButton.clicked.connect(self.page_back)
        self.BackButton_2.clicked.connect(self.page_back_2)
        self.BackButton_3.clicked.connect(self.page_back_3)
        self.NextButton.clicked.connect(self.page_next)
        self.NextButton_2.clicked.connect(self.page_next_2)
        self.HomePageButton.clicked.connect(self.home_page)


        self.GetCoffeeButton.clicked.connect(self.GetCoffee)
        self.Demo1Button.clicked.connect(self.Demo1)
        self.Demo2Button.clicked.connect(self.Demo2)
        self.Demo3Button.clicked.connect(self.Demo3)

        self.ResetRobotButton.clicked.connect(self.ResetRobot)

        
    def start_task(self):
        # goto page 2
        self.stackedWidget.setCurrentIndex(1)  
    def page_back(self):
        # page back
        self.stackedWidget.setCurrentIndex(0)
    def page_back_2(self):
        # page back 2
        self.stackedWidget.setCurrentIndex(2)
    def page_back_3(self):
        self.stackedWidget.setCurrentIndex(1)
    def page_next(self):
        # page next
        self.stackedWidget.setCurrentIndex(2)
    def page_next_2(self):
        # page next 2
        self.stackedWidget.setCurrentIndex(3)   
    def home_page(self):
        self.stackedWidget.setCurrentIndex(0)


    # def GetCoffee(self):
    #     # get coffee
    #     print("going to get a Coffee")
    #     self.move_to_pose(2.5445082421408625, 3.052429222885687, -0.0010258709257021659, 
    #                         -0.0019067729354700707, 0.0020929182724961624, 0.6739294979325221, 
    #                         0.7387902379745527) # coffee 1
    #     print("Coffee is ready")
    #     self.coffee_ready = True # set coffee_ready to True

        
    def GetCoffee(self):

        # check if coffee is hold
        if self.coffee_hold:
            print("You have already hold a coffee, you need to deliver it first !")
            return
        
        # get coffee
        coffee_positions = [
        (2.5445082421408625, 3.052429222885687),  # coffee 1
        (3.1052286713250568, 3.0866258808425804),  # coffee 2
        (3.5961160998253145, 3.127294664821698)   # coffee 3
        ]
        coffee_poses = [
        (-0.0010258709257021659, -0.0019067729354700707, 0.0020929182724961624, 0.6739294979325221, 0.7387902379745527),  # coffee 1
        (-0.0010258936361220439, -0.0021289454604649302, 0.001866538299987887, 0.7525036055487737, 0.658582042924112),  # coffee 2
        (-0.0010258727800810385, -0.0019079143092751646, 0.0020918886970050423, 0.6745581113386319, 0.7382163221522035)   # coffee 3
        ]

        coffee_status = False
    
        for index, pos in enumerate(coffee_positions):
            if self.coffee_exist[index]:  # check if coffee exist
                coffee_status = True
                print(f"going to get Coffee {index + 1}")
                self.move_to_pose(pos[0], pos[1], *coffee_poses[index]) 
                rospy.sleep(2)
                # os.system("rosservice call gazebo/delete_model \"model_name: 'coke_can__" + str(index + 1) + "'\"")


                rospy.wait_for_service('gazebo/set_model_state')
                try:
                    set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
                    model_state = ModelState()
                    model_state.model_name = f'coke_can__{index + 1}'
                    if index == 0:
                        model_state.pose.position.x = 2.500000
                        model_state.pose.position.y = 2.887800
                        model_state.pose.position.z = 0.459131
                        model_state.pose.orientation.w = 1.0
                        model_state.pose.orientation.x = 0
                        model_state.pose.orientation.y = 0
                        model_state.pose.orientation.z = 0
                        set_model_state(model_state)
                        print("Coffee_" + str(index + 1) + " is ready, click button to take it to the table " + str(index + 1))
                    elif index == 1:
                        model_state.pose.position.x = 3.120000
                        model_state.pose.position.y = 2.948130
                        model_state.pose.position.z = 0.458823
                        model_state.pose.orientation.w = 1.0
                        model_state.pose.orientation.x = 0
                        model_state.pose.orientation.y = 0
                        model_state.pose.orientation.z = 0
                        set_model_state(model_state)
                        print("Coffee_2 is ready, click button to take it to the table 3")
                    elif index == 2:
                        model_state.pose.position.x = 3.549937
                        model_state.pose.position.y = 3.028924
                        model_state.pose.position.z = 0.458980
                        model_state.pose.orientation.w = 1.0
                        model_state.pose.orientation.x = 0
                        model_state.pose.orientation.y = 0
                        model_state.pose.orientation.z = 0
                        set_model_state(model_state)
                        print("Coffee_3 is ready, click button to take it to standing person")
                except rospy.ServiceException as e:
                    print(f"Service call failed: {e}")

                # print("Coffee_" + str(index + 1) + " is ready, please take it to the table " + str(index + 1))
                self.coffee_ready = True
                self.coffee_exist[index] = False
                self.coffee_hold = True

                break

        if not coffee_status:
            print("There is no more coffees.")



    def Demo1(self):
        # demo 1
        if self.coffee_ready:
            print("going to table 1")
            self.move_to_pose(-4.300728564210335, -0.5281154777296285, -0.001025873732337007, 
                              0.0028250566991255516, 0.0001873631389456234, -0.9977696974573561, 
                              -0.06669043998623254)
            rospy.sleep(2)
            rospy.wait_for_service('gazebo/set_model_state')
            try:
                set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
                model_state = ModelState()
                model_state.model_name = f'coke_can__1'
                model_state.pose.position.x = -4.903785
                model_state.pose.position.y = -0.651438
                model_state.pose.position.z = 0.8
                model_state.pose.orientation.w = 1.0
                model_state.pose.orientation.x = 0
                model_state.pose.orientation.y = 0
                model_state.pose.orientation.z = 0
                
                set_model_state(model_state)
                print("Coffee_1 is delivered. If you want other drink, please click button More Order")
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

            self.coffee_ready = False # set coffee_ready to False
            self.coffee_hold = False
        else:
            print("You need to get a coffee first !")
        
    def Demo2(self):
        # demo 2
        if self.coffee_ready:
            print("going to table 3")
            self.move_to_pose(3.115586190223432, -0.5802054124716175, -0.0010258732281309968, 
                            0.00020367444832909562, 0.002823941875323837, -0.07130853146862916, 
                            0.9974502880889812)
            rospy.sleep(2)
            rospy.wait_for_service('gazebo/set_model_state')
            try:
                set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
                model_state = ModelState()
                model_state.model_name = f'coke_can__2'
                model_state.pose.position.x = 3.634105
                model_state.pose.position.y = -0.572636
                model_state.pose.position.z = 0.8
                model_state.pose.orientation.w = 1.0
                model_state.pose.orientation.x = 0
                model_state.pose.orientation.y = 0
                model_state.pose.orientation.z = 0
                
                set_model_state(model_state)
                print("Coffee_2 is delivered. If you want other drink, please click button More Order")
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

            self.coffee_ready = False # set coffee_ready to False
            self.coffee_hold = False
        else:
            print("You need to get a coffee first !")

    def Demo3(self):
        # demo 3
        if self.coffee_ready:
            print("going to standing person")
            self.move_to_pose(-6.713395413647411, 1.96696075640994, -0.001023541726298316, 
                              -0.0027130770702912003, 0.0007864804236439472, 0.9619010970947679, 
                              0.27338306470817825)
            rospy.sleep(2)
            os.system("rosservice call gazebo/delete_model \"model_name: 'coke_can__3' \"")
            rospy.sleep(1)
            print("Coffee_3 was taken by standing person")
 
            self.coffee_ready = False # set coffee_ready to False
            self.coffee_hold = False
        else:       
            print("You need to get a coffee first !")

    # def ResetCoffee(self):
    #     # reset coffee
    #     self.coffee_ready = False
    #     self.coffee_exist = [True, True, True] # Reset 3 coffees
    #     self.coffee_hold = False 

    #     print("Reset Done")
    def ResetRobot(self):
        # reset robot
        print("intializing Robot position...")
        self.move_to_pose(0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 
                          0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                         1.0000000000000000)
        rospy.sleep(2)
        print("Robot Reset Done")

    def move_to_pose(self,x, y, z, orientation_x, orientation_y, orientation_z, orientation_w):
        # init node
        rospy.init_node('move_robot', anonymous=True)

        # action client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
        client.wait_for_server()

        # create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom" 
        goal.target_pose.header.stamp = rospy.Time.now()


        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z

        goal.target_pose.pose.orientation.x = orientation_x
        goal.target_pose.pose.orientation.y = orientation_y
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w

        # send posse
        client.send_goal(goal)
        client.wait_for_result()

        # rospy.loginfo("Target reached!")



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


