import requests
import json
from typing import Optional, Dict, Any

class SimpleHTTPClient:
    """
    A simple HTTP client for sending and receiving data from an HTTP server
    """
    
    def __init__(self, base_url: str = "", timeout: int = 30):
        """
        Initialize the HTTP client
        
        Args:
            base_url: Base URL for all requests (optional)
            timeout: Request timeout in seconds
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.session = requests.Session()
        
        # Default headers
        self.session.headers.update({
            'User-Agent': 'SimpleHTTPClient/1.0',
            'Accept': 'application/json',
        })
    
    def _build_url(self, endpoint: str) -> str:
        """Build full URL from base URL and endpoint"""
        endpoint = endpoint.lstrip('/')
        return f"{self.base_url}/{endpoint}" if self.base_url else endpoint
    
    def set_header(self, key: str, value: str) -> None:
        """Set a header for all requests"""
        self.session.headers[key] = value
    
    def set_auth_token(self, token: str, token_type: str = "Bearer") -> None:
        """Set authorization token"""
        self.set_header('Authorization', f'{token_type} {token}')
    
    def get(self, endpoint: str = "", params: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Send GET request
        
        Args:
            endpoint: API endpoint (relative to base_url if base_url is set)
            params: Query parameters
        
        Returns:
            Dictionary containing response data and status
        """
        url = self._build_url(endpoint)
        
        try:
            response = self.session.get(url, params=params, timeout=self.timeout)
            return self._process_response(response)
        except requests.exceptions.RequestException as e:
            return self._handle_error(e)
    
    def post(self, endpoint: str = "", data: Optional[Dict] = None, json_data: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Send POST request
        
        Args:
            endpoint: API endpoint
            data: Form data to send
            json_data: JSON data to send
        
        Returns:
            Dictionary containing response data and status
        """
        url = self._build_url(endpoint)
        
        try:
            response = self.session.post(
                url, 
                data=data, 
                json=json_data, 
                timeout=self.timeout
            )
            return self._process_response(response)
        except requests.exceptions.RequestException as e:
            return self._handle_error(e)
    
    def put(self, endpoint: str = "", data: Optional[Dict] = None, json_data: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Send PUT request
        """
        url = self._build_url(endpoint)
        
        try:
            response = self.session.put(
                url, 
                data=data, 
                json=json_data, 
                timeout=self.timeout
            )
            return self._process_response(response)
        except requests.exceptions.RequestException as e:
            return self._handle_error(e)
    
    def delete(self, endpoint: str = "") -> Dict[str, Any]:
        """
        Send DELETE request
        """
        url = self._build_url(endpoint)
        
        try:
            response = self.session.delete(url, timeout=self.timeout)
            return self._process_response(response)
        except requests.exceptions.RequestException as e:
            return self._handle_error(e)
    
    def _process_response(self, response: requests.Response) -> Dict[str, Any]:
        """Process the HTTP response"""
        result = {
            'success': 200 <= response.status_code < 300,
            'status_code': response.status_code,
            'headers': dict(response.headers),
        }
        
        # Try to parse JSON, otherwise return text
        try:
            result['data'] = response.json()
        except json.JSONDecodeError:
            result['data'] = response.text
        
        return result
    
    def _handle_error(self, error: Exception) -> Dict[str, Any]:
        """Handle request errors"""
        return {
            'success': False,
            'error': str(error),
            'status_code': None,
            'data': None
        }
    
    def close(self):
        """Close the session"""
        self.session.close()


##Todo: delete following testbed
# Example usage and demonstration
if __name__ == "__main__":
    # Create client instance
    client = SimpleHTTPClient(base_url="https://jsonplaceholder.typicode.com")
    
    # Example 1: GET request - Fetch posts
    print("=== GET Request ===")
    result = client.get("/posts/1")
    print(f"Success: {result['success']}")
    print(f"Status: {result['status_code']}")
    print(f"Data: {result['data']}")
    print()
    
    # Example 2: POST request - Create new post
    print("=== POST Request ===")
    new_post = {
        "title": "Test Post",
        "body": "This is a test post",
        "userId": 1
    }
    result = client.post("/posts", json_data=new_post)
    print(f"Success: {result['success']}")
    print(f"Status: {result['status_code']}")
    print(f"Data: {result['data']}")
    print()
    
    # Example 3: PUT request - Update post
    print("=== PUT Request ===")
    updated_post = {
        "id": 1,
        "title": "Updated Title",
        "body": "Updated content",
        "userId": 1
    }
    result = client.put("/posts/1", json_data=updated_post)
    print(f"Success: {result['success']}")
    print(f"Status: {result['status_code']}")
    print(f"Data: {result['data']}")
    print()
    
    # Example 4: DELETE request
    print("=== DELETE Request ===")
    result = client.delete("/posts/1")
    print(f"Success: {result['success']}")
    print(f"Status: {result['status_code']}")
    print(f"Data: {result['data']}")
    
    # Close the client
    client.close()