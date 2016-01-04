# Car Alarm for Arduino

This alarm uses different sensors to detect activity inside the car. When any of
them sense activity, the alarm will start making noise for a particular
duration.
  
The alarm has a START_DELAY time which allow the user to arm the alarm and leave
the car. Once this delay passes, the alarm executes a routine every PERIOD_SLEEP
looking for activity in the sensors. When activity is sensed, a delay of
ALARM_DELAY is used to allow manual disconnection of the alarm. After this time
the alarm starts making noise during ALARM_DURATION time. After this time the
alarm waits REARM_DELAY before rearming alarm again.

  
Setup procedure: depending on alarm battery level the starting procedure
changes, indicating if everything is ok to work, if the battery needs to be
replaced/charged, or if battery is so low that the system won't run:
  
  1. Correct function is advertised by BATTERY_OK_REPETITIONS short buzzes.
  
  2. Low battery is advertised by BATTERY_ALERT_REPETITIONS long buzzes, but the
     alarm stills running.

  3. Very low battery is advertised by BATTERY_ERROR_REPETITIONS short buzzes
     and the system won't run.
