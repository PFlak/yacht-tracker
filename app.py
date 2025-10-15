import http.server
import socketserver
import webbrowser
import threading
import os

frontend_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend")
os.chdir(frontend_dir)

def start_server():
    Handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", 8000), Handler) as httpd:
        print(f"Serwer działa na http://localhost:8000")
        httpd.serve_forever()

# Uruchomienie serwera w osobnym wątku
threading.Thread(target=start_server, daemon=True).start()

webbrowser.open("http://localhost:8000")

input("Naciśnij Enter, aby zakończyć serwer...\n")
