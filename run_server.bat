@echo off
cd /d "%~dp0"
echo LD2450 server starting...
echo Local: http://127.0.0.1:8000/
echo LAN  : http://192.168.1.128:8000/
python -m uvicorn server.app:app --host 0.0.0.0 --port 8000 --reload
