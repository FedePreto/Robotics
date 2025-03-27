%% Parameter server

ptree = rosparam;
ptree.AvailableParameters

r_color = get(ptree,'/turtlesim/background_r')
g_color = ptree.get('/turtlesim/background_g')
b_color = ptree.get('/turtlesim/background_b')

% We change the background color of the turtlesim 
% orange: rgb(255,172,28)

ptree.set('/turtlesim/background_r',255);
ptree.set('/turtlesim/background_g',172);
ptree.set('/turtlesim/background_b',28);

%ptree.del(paramname)
%ptree.has(paramname)