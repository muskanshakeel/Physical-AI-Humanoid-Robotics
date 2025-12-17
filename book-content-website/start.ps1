Write-Host "Starting Book Content Assistant Website..." -ForegroundColor Green
Write-Host ""

# Check if Node.js is installed
if (Get-Command node -ErrorAction SilentlyContinue) {
    Write-Host "Node.js is installed." -ForegroundColor Green

    # Change to the website directory
    Set-Location -Path "D:\Q4_hackathon\book-content-website"

    # Install dependencies if needed
    Write-Host "Installing dependencies if needed..." -ForegroundColor Yellow
    npm install *>$null

    # Start the server in a new process
    Write-Host "Starting server on port 8080..." -ForegroundColor Yellow
    Start-Process "http://localhost:8080"
    npx http-server -p 8080
} else {
    Write-Host "Node.js is not installed. Please install Node.js first." -ForegroundColor Red
    Write-Host "Visit https://nodejs.org/ to download and install Node.js" -ForegroundColor Red
    Read-Host "Press Enter to exit"
}