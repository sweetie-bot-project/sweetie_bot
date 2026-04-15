from os import environ

class Config(object):
    DEBUG = False

class ProductionConfig(Config):
    API_KEY = environ.get("API_KEY")
    HOST = environ.get("HOST", "0.0.0.0")
    PORT = int(environ.get("PORT", "5005"))

class DevelopmentConfig(Config):
    API_KEY = "test_api_key"
    DEBUG = True
    HOST = environ.get("HOST", "127.0.0.1")
    PORT = int(environ.get("PORT", "5005"))
