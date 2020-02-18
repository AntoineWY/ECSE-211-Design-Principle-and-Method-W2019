# ECSE-211-Design-Principle-and-Method-W2019
**Overview**<br>
The goal of this project is to design a machine that can autonomously navigate a closed course in search of 210 ml soft drink cans for recycling.  Here two robots will compete with one another to determine who can recover the most re-cyclable material in agiven time interval.  Cans come in two weights with four different colors for a total of 8 different color-weight combinations, with the heavier cans being more “valuable”.Points are awarded for each can correctly retrieved (i.e. with the right color), with double points for the heavier cans. <br>

Specific Details:<br>
The WiFi class delivers the game parameters which are summarized in the following section.  The procedure that each player must follow is summarized in the following steps and mustbe adhered to:<br>
1.Each robot is placed in the corner specified by the marshal running the competition round.Player will be instructed as to where to place and orient the machine.<br>
2.Once placed and the start button pushed, the player are no longer permitted to touch the machine.  If there is any contact with the machine the team is disqualified for that round.<br>
3.One started, the machine waits for the game server to deliver the parameters for the current run.  This is done through a method call which will block until complete.<br>
4.Each machine localizes to the grid.  When completed, the machine must stop and issue a sequence of 3 beeps.<br>
5.Each machine navigates to their correspondingtunnel, transits, andthen proceeds to their search zone.  Upon arriving, each machine will again stop and issue a sequence of 3 beeps.<br>
6.Each machine begins the search process.  Upon detecting a can, the machine stops,assesses the can (color and weight),and issues one or more beeps based on the color of the cans.<br>

Complete view of the final project please see ![project requirement document]()
