%% Simple ROS Publisher in MATLAB
% LTC Steve Crews
% 10 Feb 2022

% publishes some matrix 


%%

clear; close all; format compact; clc;

% Use this code to test Matlab's ability to talk with ROS after you complete the following two tutorials
% 1) Writing a Simple Publisher and Subscriber (Python) https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
% 2) Examining the Simple Publisher and Subscriber https://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

%% SET UP ROS IN MATLAB

% Ensure MATLAB starts interacting with ROS
% rosinit % only run this in matlab once, likely in command line


% % % Create a Publisher (send message to ROS)

% let's publish an 'array' message in Matlab
matlab_pub1 = rospublisher('/Bflat_mat','std_msgs/Float64MultiArray');% https://www.mathworks.com/help/ros/ref/subscriber.html#d122e23186

% package the message 
matlab_message = rosmessage( matlab_pub1 ) ;

pause(1) % let MATLAB create the subscriber before asking for the talker


%% In a loop, talk to ROS 


for i = 1:10000 % like forever

    % Get rostime seconds and nanoseconds, turn into a single time number
    [time,issimtime] = rostime("now") ;
    ROS_time = time.Sec + time.Nsec/1e9 ;
    
    % put some data in an array
    Bmatrix      = [ ROS_time , 15 , 14 , 13 ;
                           12 , 11 , 10 ,  9 ;
                            8 ,  7 ,  6 ,  5 ;
                            4 ,  3 ,  2 ,  1 ] ;
    Bflat = reshape(Bmatrix',1,16) ;
    % I like to always include the time of the message in the message so
    % you can see if it's stale or not
    
    matlab_message.Data = Bflat ;
    send( matlab_pub1 , matlab_message )
    
    pause( 2) % pause 2 seconds, long enough to see a difference
    
end


% AT SOME POINT, GO BACK TO TERMINAL AND LISTEN TO MATLAB

% $ rostopic list 
% you should see /matlab_talker as a ROS topic

% $ rostopic echo /matlab_chatter
% now you should see the array you created in matlab 
% with ROS still listening, run this section again "CTRL" + "ENTER"


