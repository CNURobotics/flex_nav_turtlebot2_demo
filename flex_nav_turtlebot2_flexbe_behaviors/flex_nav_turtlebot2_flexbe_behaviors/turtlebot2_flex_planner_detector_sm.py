#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ball_detector_flexbe_states.ball_detector_state import BallDetectorState
from flex_nav_flexbe_states.clear_costmaps_state import ClearCostmapsState
from flex_nav_flexbe_states.follow_path_state import FollowPathState
from flex_nav_flexbe_states.get_path_by_name_state import GetPathByNameState
from flex_nav_flexbe_states.get_path_state import GetPathState
from flex_nav_flexbe_states.get_pose_state import GetPoseState
from flex_nav_flexbe_states.rotate_angle_state import RotateAngleState
from flex_nav_flexbe_states.timed_twist_state import TimedTwistState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat April 16 2022
@author: Josh Zutell, David Conner
'''
class Turtlebot2FlexPlannerDetectorSM(Behavior):
    '''
    Uses Flexible Navigation to control the Turtlebot 2 robot with Ball Detector and Pre-planned route
    '''


    def __init__(self, node):
        super(Turtlebot2FlexPlannerDetectorSM, self).__init__()
        self.name = 'Turtlebot2 Flex Planner Detector'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        BallDetectorState.initialize_ros(node)
        ClearCostmapsState.initialize_ros(node)
        FollowPathState.initialize_ros(node)
        GetPathByNameState.initialize_ros(node)
        GetPathState.initialize_ros(node)
        GetPoseState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)
        RotateAngleState.initialize_ros(node)
        TimedTwistState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:943 y:286, x:1073 y:20
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

		# [/MANUAL_CREATE]

        # x:620 y:107, x:621 y:217, x:230 y:365, x:526 y:472, x:640 y:322, x:637 y:255, x:624 y:375, x:314 y:426, x:594 y:416, x:599 y:44, x:1030 y:365, x:1130 y:365
        _sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'red_ball', 'other_ball'], input_keys=['plan'], conditions=[
                                        ('finished', [('low_level_planner', 'done')]),
                                        ('failed', [('low_level_planner', 'failed')]),
                                        ('other_ball', [('ball_detector', 'green_ball')]),
                                        ('other_ball', [('ball_detector', 'blue_ball')]),
                                        ('red_ball', [('ball_detector', 'red_ball')]),
                                        ('failed', [('ball_detector', 'unavailable')]),
                                        ('failed', [('ball_detector', 'invalid')]),
                                        ('failed', [('low_level_planner', 'canceled')])
                                        ])

        with _sm_container_0:
            # x:189 y:104
            OperatableStateMachine.add('low_level_planner',
                                        FollowPathState(topic='low_level_planner'),
                                        transitions={'done': 'finished', 'failed': 'failed', 'canceled': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off},
                                        remapping={'plan': 'plan'})

            # x:188 y:198
            OperatableStateMachine.add('ball_detector',
                                        BallDetectorState(balls_topic='/ball_detector/balls', min_radius_pixels=-1.0),
                                        transitions={'unavailable': 'failed', 'invalid': 'failed', 'red_ball': 'red_ball', 'green_ball': 'other_ball', 'blue_ball': 'other_ball'},
                                        autonomy={'unavailable': Autonomy.Off, 'invalid': Autonomy.Off, 'red_ball': Autonomy.Off, 'green_ball': Autonomy.Off, 'blue_ball': Autonomy.Off})



        with _state_machine:
            # x:193 y:26
            OperatableStateMachine.add('ClearCostmap',
                                        ClearCostmapsState(costmap_topics=['high_level_planner/clear_costmap','low_level_planner/clear_costmap'], timeout=5.0),
                                        transitions={'done': 'Receive Goal', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:462 y:389
            OperatableStateMachine.add('Container',
                                        _sm_container_0,
                                        transitions={'finished': 'Continue', 'failed': 'Continue', 'red_ball': 'saw_red', 'other_ball': 'saw_other_ball'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'red_ball': Autonomy.Inherit, 'other_ball': Autonomy.Inherit},
                                        remapping={'plan': 'plan'})

            # x:435 y:146
            OperatableStateMachine.add('Continue',
                                        OperatorDecisionState(outcomes=["yes","no","clearcostmap", "patrol"], hint="Continue planning to new goal?", suggestion="yes"),
                                        transitions={'yes': 'Receive Goal', 'no': 'finished', 'clearcostmap': 'ClearCostmap', 'patrol': 'PatrolPath'},
                                        autonomy={'yes': Autonomy.Full, 'no': Autonomy.Full, 'clearcostmap': Autonomy.Full, 'patrol': Autonomy.Full})

            # x:194 y:301
            OperatableStateMachine.add('ExecutePlan',
                                        OperatorDecisionState(outcomes=["yes","no"], hint="Execute the current plan?", suggestion="yes"),
                                        transitions={'yes': 'Container', 'no': 'Continue'},
                                        autonomy={'yes': Autonomy.High, 'no': Autonomy.Full})

            # x:960 y:70
            OperatableStateMachine.add('Log Recovered',
                                        LogState(text="Re-plan after recovery", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'New Plan'},
                                        autonomy={'done': Autonomy.Off})

            # x:744 y:355
            OperatableStateMachine.add('Log Success',
                                        LogState(text="Success!", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Continue'},
                                        autonomy={'done': Autonomy.Off})

            # x:728 y:65
            OperatableStateMachine.add('New Plan',
                                        GetPathState(planner_topic="high_level_planner"),
                                        transitions={'planned': 'Container', 'empty': 'Receive Goal', 'failed': 'Continue'},
                                        autonomy={'planned': Autonomy.Off, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'goal': 'goal', 'plan': 'plan'})

            # x:248 y:409
            OperatableStateMachine.add('PatrolPath',
                                        GetPathByNameState(action_server_name='get_path_by_name', path_name='creech_patrol'),
                                        transitions={'success': 'Container', 'empty': 'Continue', 'failed': 'Continue'},
                                        autonomy={'success': Autonomy.Off, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'plan': 'plan'})

            # x:203 y:113
            OperatableStateMachine.add('Receive Goal',
                                        GetPoseState(topic='flex_nav_global/goal'),
                                        transitions={'done': 'Receive Path'},
                                        autonomy={'done': Autonomy.Low},
                                        remapping={'goal': 'goal'})

            # x:205 y:207
            OperatableStateMachine.add('Receive Path',
                                        GetPathState(planner_topic="high_level_planner"),
                                        transitions={'planned': 'ExecutePlan', 'empty': 'Continue', 'failed': 'Continue'},
                                        autonomy={'planned': Autonomy.Off, 'empty': Autonomy.Low, 'failed': Autonomy.Low},
                                        remapping={'goal': 'goal', 'plan': 'plan'})

            # x:593 y:592
            OperatableStateMachine.add('saw_other_ball',
                                        LogState(text='Saw a blue or green ball', severity=Logger.REPORT_HINT),
                                        transitions={'done': 'spin'},
                                        autonomy={'done': Autonomy.Off})

            # x:599 y:489
            OperatableStateMachine.add('saw_red',
                                        LogState(text='Saw a red ball', severity=Logger.REPORT_HINT),
                                        transitions={'done': 'spin_right'},
                                        autonomy={'done': Autonomy.Off})

            # x:750 y:599
            OperatableStateMachine.add('spin',
                                        TimedTwistState(target_time=20, velocity=0.01, rotation_rate=0.628, cmd_topic='cmd_vel'),
                                        transitions={'done': 'Continue'},
                                        autonomy={'done': Autonomy.Off})

            # x:745 y:484
            OperatableStateMachine.add('spin_right',
                                        RotateAngleState(target_time=20.0, target_angle=-360.0, cmd_topic='/cmd_vel', odometry_topic='/odom'),
                                        transitions={'done': 'Continue'},
                                        autonomy={'done': Autonomy.Off})

            # x:875 y:162
            OperatableStateMachine.add('AutoReplan',
                                        OperatorDecisionState(outcomes=["yes","no"], hint="Re-plan to current goal?", suggestion="yes"),
                                        transitions={'yes': 'Log Recovered', 'no': 'Continue'},
                                        autonomy={'yes': Autonomy.High, 'no': Autonomy.Full})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

	# [/MANUAL_FUNC]
