#!/bin/bash
#author:wuyk
#date:20171123

HD330PLUS=hd330plus
HD330PLUS_ADD=.20
HD550=hd550
HD550_ADD=.50
HD400=hd400
HD400_ADD=.40
if [ "${BRANCH}" == "trunk" ]; then
	UPDATE_FILE_PATH=${UPDATE_FILE_PATH}/$BRANCH/app/PackManu
elif [[ ${BRANCH} == br-* ]];then
	if [ "$TYPE" == "$HD330PLUS" ];then
		UPDATE_FILE_PATH=${UPDATE_FILE_PATH}/branches/$BRANCH${HD330PLUS_ADD}/app/PackManu
        elif [ "$TYPE" = "$HD550" ];then
		UPDATE_FILE_PATH=${UPDATE_FILE_PATH}/branches/$BRANCH${HD550_ADD}/app/PackManu
        elif [ "$TYPE" = "$HD400" ];then
		UPDATE_FILE_PATH=${UPDATE_FILE_PATH}/branches/$BRANCH${HD400_ADD}/app/PackManu
        else
		UPDATE_FILE_PATH=${UPDATE_FILE_PATH}/branches/$BRANCH/app/PackManu
	fi
	echo $UPDATE_FILE_PATH >/root/test
fi

