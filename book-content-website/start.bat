@echo off
echo Starting Book Content Assistant Website...
echo.

REM Check if Node.js is installed
node --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Node.js is not installed. Please install Node.js first.
    echo Visit https://nodejs.org/ to download and install Node.js
    pause
    exit /b 1
)

echo Node.js is installed.
echo Installing dependencies if needed...
npm install >nul 2>&1

echo.
echo Opening website in your default browser...
start http://localhost:8080

echo.
echo Starting server on port 8080...
npx http-server -p 8080

pause