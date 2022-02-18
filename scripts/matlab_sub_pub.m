%% Simple ROS Subscriber & Publisher in MATLAB
% LTC Steve Crews
% 10 Feb 2022

% subscribes to both python and matlab messages
% completes some math operations 
% publishes result of those operations

%%

clear; close all; format compact; clc;

% Use this code to test Matlab's ability to talk with ROS after you complete the following two tutorials
% 1) Writing a Simple Publisher and Subscriber (Python) https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
% 2) Examining the Simple Publisher and Subscriber https://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

%% SET UP ROS IN MATLAB

% Ensure MATLAB starts interacting with ROS
% rosinit % only run this in matlab once, likely in command line


% % % Create a Subscriber (listen to ROS)

% python and matlab are publishing 
Aflat_sub = rossubscriber('/Aflat_py','std_msgs/Float64MultiArray');% https://www.mathworks.com/help/ros/ref/subscriber.html#d122e23186
Bflat_sub = rossubscriber('/Bflat_mat','std_msgs/Float64MultiArray');

% % % Create a Publisher (send message to ROS)

% let's publish an 'array' message in Matlab
Cflat_pub = rospublisher('/Cflat_mat','std_msgs/Float64MultiArray');% https://www.mathworks.com/help/ros/ref/subscriber.html#d122e23186
Dflat_pub = rospublisher('/Dflat_mat','std_msgs/Float64MultiArray');

% package the messages 
Cflat_message = rosmessage( Cflat_pub );
Dflat_message = rosmessage( Dflat_pub );

pause(1) % let MATLAB create the subscriber before asking for the talker


%% In a loop, listen to ROS and print the message and date/time
%
tic
while toc < 200

    %% GET LATEST MESSAGES: /Aflat_py, /Bflat_mat
    Aflat = Aflat_sub.LatestMessage.Data ;% get flattened matrix
    A     = reshape(Aflat,4,4)'          ;%form into 4x4
    
    Bflat = Bflat_sub.LatestMessage.Data ;%get flattened matrix   
    B     = reshape(Bflat,4,4)'          ;%reshape into 4x4
    
    
    %% DO SOME OPERATIONS
    C = A+B  
    D = A*B
    
    
    %% PUBLISH MESSAGES
    Cflat = reshape(C,1,16)          ;%flatten into array
    Cflat_message.Data = Cflat       ;%save in message structure
    send( Cflat_pub , Cflat_message ) %send /Cflat_mat

    Dflat_message.Data = reshape(D,1,16) ;%flatten and save into message
    send( Dflat_pub , Dflat_message ) %send /Dflat_mat
    
    
    %% 
    pause(1) % pause so somewhat readable
end
%}



%% In a loop, talk to ROS 
%
%     ROS_time_seconds = str2double(python_msg(13:end)) ;% ROS time in seconds
%     ROS_datetime_zulu = datetime(1970,1,1) + seconds(ROS_time_seconds) ;% ROS time as a date 
%     disp(ROS_datetime_zulu) %print the date/time
% 
% for i = 1:10
% 
%     % Get rostime seconds and nanoseconds, turn into a single time number
%     [time,issimtime] = rostime("now") ;
%     ROS_time = time.Sec + time.Nsec/1e9 ;
%     
%     % put some data in an array
%     matlab_array = [ ROS_time , i , 5*i , 10*i^2 , 15*i^3 , 20*i^4 ] 
%     % I like to always include the time of the message in the message so
%     % you can see if it's stale or not
%     
%     matlab_message.Data = matlab_array ;
%     send( matlab_talker , matlab_message )
%     
%     pause( 2) % pause 2 seconds, long enough to see a difference
%     
% end

% AT SOME POINT, GO BACK TO TERMINAL AND LISTEN TO MATLAB

% $ rostopic list 
% you should see /matlab_talker as a ROS topic

% $ rostopic echo /matlab_chatter
% now you should see the array you created in matlab 
% with ROS still listening, run this section again "CTRL" + "ENTER"


%}