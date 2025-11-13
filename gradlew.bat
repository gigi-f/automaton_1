@echo off
REM Minimal gradlew shim for Windows: delegates to system 'gradle' if present.
where gradle >nul 2>&1
if %ERRORLEVEL%==0 (
  gradle %*
) else (
  echo Gradle not found. Please install Gradle or add a proper Gradle wrapper (gradle-wrapper.jar).
  exit /b 1
)
