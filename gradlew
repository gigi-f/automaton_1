#!/bin/sh
# Minimal gradlew shim: delegates to system 'gradle' if present.
# This is a lightweight helper so users can run './gradlew :desktop:run' when
# a full gradle wrapper binary isn't checked in. To get a true wrapper,
# run 'gradle wrapper' locally and check in 'gradle/wrapper/gradle-wrapper.jar'.

if command -v gradle >/dev/null 2>&1; then
  exec gradle "$@"
else
  echo "Gradle not found. Please install Gradle or add a proper Gradle wrapper (gradle-wrapper.jar)."
  exit 1
fi
