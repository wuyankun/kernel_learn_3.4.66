#!/bin/bash
#author:wuyk
#date:20171123

test_print()
{
	if [ -n "$1" ];then
		echo $1	    #enable test echo
	fi	
	echo -n ""
}

cd data
if [ -n "$1" ];then
	TYPE=$1
else
	TYPE=S50
fi

if [ "$TYPE" = "S50" ];then
	ALL_TYPE_DIR=`ls|grep "_S50$"|sort`
elif [ "$TYPE" = "P50" ];then
	ALL_TYPE_DIR=`ls|grep "_P50$"|sort`
elif [ "$TYPE" = "S40plus" ];then
	ALL_TYPE_DIR=`ls|grep "_S40plus$"|sort`
elif [ "$TYPE" = "S45" ];then
	ALL_TYPE_DIR=`ls|grep "_S45$"|sort`
elif [ "$TYPE" = "S50cv" ];then
	ALL_TYPE_DIR=`ls|grep "_S50cv$"|sort`
elif [ "$TYPE" = "S50exp" ];then
	ALL_TYPE_DIR=`ls|grep "_S50exp$"|sort`
elif [ "$TYPE" = "S50pro" ];then
	ALL_TYPE_DIR=`ls|grep "_S50pro$"|sort`
elif [ "$TYPE" = "S50vo" ];then
	ALL_TYPE_DIR=`ls|grep "_S50vo$"|sort`
fi
LAST_TYPE_VERSION=`echo $ALL_TYPE_DIR|awk '{print $NF}'`
test_print $LAST_TYPE_VERSION

if [ -n "$LAST_TYPE_VERSION" ];then
	test_print "Found It!"
else
	test_print "Not Found It!"
fi

exit 0
