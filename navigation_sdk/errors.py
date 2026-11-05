class NavigationError(Exception):
    def __init__(self, code, message, *, status=409, details=None):
        super().__init__(message)
        self.code, self.message, self.status = code, message, status
        self.details = details or {}

    def to_dict(self):
        return {"error": {"code": self.code, "message": self.message,
                          "details": self.details}}
