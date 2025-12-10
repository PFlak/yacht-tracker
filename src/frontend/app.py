from flask import Flask
from routes import init_routes

app = Flask(
    __name__, 
    template_folder="templates", 
    static_folder="static"
)

app.secret_key = "super_secret_key_change_me"

# załaduj routes
init_routes(app)

if __name__ == "__main__":
    app.run(debug=True, port=8000)
