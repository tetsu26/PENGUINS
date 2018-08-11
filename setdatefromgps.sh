#!/bin/sh
echo start at `/bin/date`
for i in 5 5; do
  echo sleep $i
  /bin/sleep $i
  GPSDATE="`/usr/bin/gpspipe -w | /usr/bin/head -10 | /bin/grep TPV | /bin/sed -r 's/.*"time":"([^"]*)".*/\1/' | /usr/bin/head -1`"
  echo $GPSDATE
  /bin/date -s "$GPSDATE"
done
echo end at `/bin/date`
