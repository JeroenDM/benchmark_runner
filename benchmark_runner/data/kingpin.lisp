(variables
(config home 0 0 0 0 0 0)
(pose start 0.85 -0.145 0.0 0.65328148 0.65328148 0.27059805 0.27059805)
(pose stop 0.85 0.145 0.0 0.65328148 0.65328148 0.27059805 0.27059805))

(commands
(movep start)
(movelin stop :constraints :c1)
(movej home))

(constraints
(con c1 relative rotation tolerance z -3.14159265359 3.14159265359))