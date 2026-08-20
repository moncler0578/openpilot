#!/usr/bin/bash

TIMEZONE="Asia/Seoul"
GET_PROP_ATZ=$(getprop persist.sys.timezone)

if [ "$GET_PROP_ATZ" != "$TIMEZONE" ]; then
    setprop persist.sys.timezone "$TIMEZONE"
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

