%Inverse Kinematics for 6 DOF robot arm

%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
L(1) = Link([ 0     0.2     0          -pi/2    0        0], 'standard');
L(2) = Link([ 0 	0.      0.2        0        0        0], 'standard');
L(3) = Link([ 0     0.      0         pi/2    0         pi/2], 'standard');
L(4) = Link([ 0     0.4     0        -pi/2    0         pi/2], 'standard');
L(5) = Link([ 0     0.0     0         pi/2    0         0   ], 'standard');
L(6) = Link([ 0     0.4     0          0      0         0], 'standard');

robot = SerialLink(L);

% some useful poses
qz = [0 0 0 0 0 0]; % zero angles, L shaped pose

%number of tests
num_tests = 10;

%generating random joint angles with joint limits
jt_angles = random('unif', -pi, pi/2, 6, num_tests);

tolerance = 1e-3;
q1 = zeros(6,num_tests);
q2 = q1;
for i=1:num_tests
    T_des = robot.fkine(jt_angles(:,i));
    q_init1 = [pi/2;pi/2;pi/2;pi/2;pi/2;pi/2];
    q_init2 = [0;0;0;0;0;0];
    q1(1:6,i) = q_init1;
    kd = .0001;
    K = eye(3);
    delta_t = .1;
    iterations = 0;
    while true
        J = robot.jacob0(q1(:,i)); % calc jacobian
        J = J(1:3,:);
        e = tr2delta(robot.fkine(q1(:,i)),T_des);
        e = e(1:3);
        if norm(e) < tolerance
            display(q1(:,i))
            break;
        end
        
        q1(:,i) = q1(:,i) + J'*inv(J*J'+kd^2*eye(3))*(K*e)*delta_t;
        iterations = iterations + 1;
        if(iterations > 2000)
            display('Solution not found')
            q1(:,i) = q_init1;
            break
        end
    end
    
    while true
        J = robot.jacob0(q2(:,i)); % calc jacobian
        J = J(1:3,:);
        e = tr2delta(robot.fkine(q2(:,i)),T_des);
        e = e(1:3);
        if norm(e) < tolerance
            display(q2(:,i))
            break;
        end
        
        q2(:,i) = q2(:,i) + J'*inv(J*J'+kd^2*eye(3))*(K*e)*delta_t;
        iterations = iterations + 1;
        if(iterations > 2000)
            display('Solution not found')
            q2(:,i) = q_init2;
            break
        end
    end
end

%plot 3rd entry
q1p = q1(:,3)
q2p = q2(:,3)
scatter(1:6,q1p', 'k', 'filled')
hold on
scatter(1:6, q2p', 'k')
scatter(1:6, jt_angles(:,3)', 'k', 'd')
xlabel('Joint Number')
ylabel('Joint Angle (Radians)')
legend('Zero Initial Config', 'Pi/2 Initial Config', 'Randomly Generated Joint Angles')