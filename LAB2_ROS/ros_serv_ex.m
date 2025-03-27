clc
clear 
close all

rosip = "192.168.240.206";
rosport = 11311;
rosinit(rosip,rosport);

rosservice list

%% Kill the turtle
kill_client = rossvcclient("/kill", "DataFormat","struct");
kill_msg = rosmessage(kill_client);
kill_msg.Name = 'turtle1';

kill_resp = call(kill_client,kill_msg,"Timeout",3);

%% Spawn a new turtle

spawn_client = rossvcclient("/spawn","DataFormat","struct");
spawn_msg = rosmessage(spawn_client);
spawn_msg.X = single(1);
spawn_msg.Y = single(1);
spawn_msg.Theta = single(3.14);
spawn_msg.Name = 'Pippo';
spawn_resp = call(spawn_client,spawn_msg,"Timeout",3)

rostopic list %check allowed operation 

% E1

prompt = 'Insert the value for x';
x = input(prompt);
prompt = 'Insert the value for y';
y = input(prompt);
prompt = 'Insert the value for theta';
theta = input(prompt);

teleport_client = rossvcclient("/teleport_absolute","DataFormat","struct");




%% Service to add two numbers
rosmsg show rospy_tutorials/AddTwoIntsRequest
rosmsg show rospy_tutorials/AddTwoIntsResponse

rosservice list  

%Now we add a new service
rossvcserver("/sum","roscpp_tutorials/TwoInts", @sumTwonumbersCallback,"DataFormat","struct");

rosservice list %We should see that the new service /sum is add to the list

sum_client = rossvcclient("/sum","DataFormat","struct");
sum_msg = rosmessage(sum_client);

sum_msg.A = int64(3);
sum_msg.B = int64(4);

sum_resp = call(sum_client,sum_msg,"Timeout",3);

