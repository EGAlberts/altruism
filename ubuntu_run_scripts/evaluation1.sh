for EXP in 1 2 3 4 5
do
      ./demo_onlyid.sh
      
      while [ ! -f ~/nav2simple_ws/mission.done ]
      do
      	  echo "waiting for mission to be done"              
          sleep 10 #sustainability!
      done
      echo "mission done"
      rm ~/nav2simple_ws/mission.done
      ./killall.sh

done
