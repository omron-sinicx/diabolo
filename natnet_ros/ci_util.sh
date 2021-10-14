# From https://github.com/ros-planning/natnet_ros_ci/blob/master/util.sh

#********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, University of Colorado, Boulder
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Univ of CO, Boulder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/

# Author: Dave Coleman <dave@dav.ee>
# Desc: Utility functions used to make CI work better in Travis

#######################################
export TRAVIS_FOLD_COUNTER=1


#######################################
# Start a Travis fold with timer
#
# Arguments:
#   travis_fold_name: name of line
#   command: action to run
#######################################
function travis_time_start {
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    COMMAND=${@:2} # all arguments except the first

    # Start fold
    echo -e "\e[0Ktravis_fold:start:$TRAVIS_FOLD_NAME"
    # Output command being executed
    echo -e "\e[0Ktravis_time:start:$TRAVIS_TIME_ID\e[34m$COMMAND\e[0m"
}

#######################################
# Wraps up the timer section on Travis CI (that's started mostly by travis_time_start function).
#
# Arguments:
#   travis_fold_name: name of line
#######################################
function travis_time_end {
    if [ -z $TRAVIS_START_TIME ]; then
        echo '[travis_time_end] var TRAVIS_START_TIME is not set. You need to call `travis_time_start` in advance. Rerunning.';
        return;
    fi
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))

    # Output Time
    echo -e "travis_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\e[0K"
    # End fold
    echo -e -n "travis_fold:end:$TRAVIS_FOLD_NAME\e[0m"

    unset $TRAVIS_FOLD_NAME
}

#######################################
# Display command in Travis console and fold output in dropdown section
#
# Arguments:
#   command: action to run
#######################################
function travis_run() {
  local command=$@

  #echo -e "\e[0Ktravis_fold:start:command$TRAVIS_FOLD_COUNTER \e[34m$ $command\e[0m"
  travis_time_start natnet_ros_ci$TRAVIS_FOLD_COUNTER $command
  # actually run command
  $command || exit 1 # kill build if error
  travis_time_end natnet_ros_ci$TRAVIS_FOLD_COUNTER
  #echo -e -n "\e[0Ktravis_fold:end:command$TRAVIS_FOLD_COUNTER\e[0m"

  let "TRAVIS_FOLD_COUNTER += 1"
}

#######################################
# Same as travis_run except ignores errors and does not break build
function travis_run_true() {
  local command=$@

  travis_time_start natnet_ros_ci$TRAVIS_FOLD_COUNTER $command
  # actually run command
  $command # ignore errors
  travis_time_end natnet_ros_ci$TRAVIS_FOLD_COUNTER

  let "TRAVIS_FOLD_COUNTER += 1"
}

#######################################
# Orginal version from: https://github.com/travis-ci/travis-build/blob/d63c9e95d6a2dc51ef44d2a1d96d4d15f8640f22/lib/travis/build/script/templates/header.sh
function my_travis_wait() {
  local timeout=$1

  if [[ $timeout =~ ^[0-9]+$ ]]; then
    # looks like an integer, so we assume it's a timeout
    shift
  else
    # default value
    timeout=20
  fi

  # Show command in console before running
  echo -e "\e[34m$ $@\e[0m"

  my_travis_wait_impl $timeout "$@"
}

#######################################
function my_travis_wait_impl() {
  local timeout=$1
  shift

  local cmd="$@"
  local log_file=my_travis_wait_$$.log

  $cmd 2>&1 >$log_file &
  local cmd_pid=$!

  my_travis_jigger $! $timeout $cmd &
  local jigger_pid=$!
  local result

  {
    wait $cmd_pid 2>/dev/null
    result=$?
    ps -p$jigger_pid 2>&1>/dev/null && kill $jigger_pid
  } || return 1

  echo -e "\nThe command \"$cmd\" exited with $result."
  #echo -e "\n\033[32;1mLog:\033[0m\n"
  cat $log_file

  return $result
}

#######################################
function my_travis_jigger() {
  # helper method for travis_wait()
  local cmd_pid=$1
  shift
  local timeout=$1 # in minutes
  shift
  local count=0


  # clear the line
  echo -e "\n"

  while [ $count -lt $timeout ]; do
    count=$(($count + 1))
    echo -ne "Still running ($count of $timeout min): $@\r"
    sleep 60
  done

  echo -e "\n\033[31;1mTimeout (${timeout} minutes) reached. Terminating \"$@\"\033[0m\n"
  kill -9 $cmd_pid
}
