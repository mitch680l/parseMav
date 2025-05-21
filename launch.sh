#!/usr/bin/env bash
#
# Usage: ./launch.sh {test|test_reader|main}

set -e

cd "$(dirname "$0")"
BINDIR="$PWD/bin"

start_reader()    { screen -dmS parseMav_reader    "$BINDIR/reader";      }
start_test_mav()  { screen -dmS parseMav_test_mav  "$BINDIR/test_mav";   }
start_consumer()  { screen -dmS parseMav_consumer  "$BINDIR/consumer";   }
start_interpreter(){ screen -dmS parseMav_interpreter "$BINDIR/interpreter"; }
start_speaker()   { screen -dmS parseMav_speaker   "$BINDIR/speaker";    }

case "$1" in
  test)
    start_reader
    start_test_mav
    ;;
  test_reader)
    start_reader
    start_consumer
    ;;
  main)
    start_reader
    start_interpreter
    start_speaker
    ;;
  *)
    echo "Usage: $0 {test|test_reader|main}"
    exit 1
    ;;
esac

echo "Launched session: $1"
