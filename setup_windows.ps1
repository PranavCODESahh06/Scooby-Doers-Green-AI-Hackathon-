$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
py -3 -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r requirements.txt
Write-Host "Setup complete. Open the folder in VS Code, select .venv Python, run AquaMind_FINAL.py."
