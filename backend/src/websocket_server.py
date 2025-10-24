import asyncio
import websockets
import json
import logging

class WebSocketServer:
    """
    Sends robot data to connected web clients.
    Runs in separate thread alongside ROS nodes.
    """
    
    def __init__(self, data_manager, host='localhost', port=8765, rate=10):
        self.logger = logging.getLogger(__name__)
        self.data_manager = data_manager
        self.host = host
        self.port = port
        
        safe_rate = max(1, int(rate) if isinstance(rate, (int, float)) else 10)
        self.update_interval = 1.0 / safe_rate
        
        self.clients = set()
        
        self.logger.info(f'WebSocket configured: {host}:{port} @ {safe_rate}Hz')
    
    async def register(self, websocket):
        """Add new client connection."""
        self.clients.add(websocket)
        self.logger.info(f'Client connected. Total: {len(self.clients)}')
    
    async def unregister(self, websocket):
        """Remove client connection."""
        self.clients.discard(websocket)
        self.logger.info(f'Client disconnected. Total: {len(self.clients)}')
    
    async def send_data(self, websocket):
        """Send data to one client continuously."""
        
        while True:
            try:
                data = self.data_manager.get_latest_data()
                await websocket.send(json.dumps(data))
                await asyncio.sleep(self.update_interval)
            except websockets.exceptions.ConnectionClosed:
                self.logger.info("Client connection closed. Stopping data send task for this client.")
                break
            except Exception as e:
                self.logger.error(f"Unexpected error in send_data: {e}")
                break
    
    async def handler(self, websocket):
        """Handle client connection lifecycle."""
        await self.register(websocket)
        try:
            await self.send_data(websocket)
        except Exception as e:
            self.logger.error(f'Error in handler: {e}')
        finally:
            await self.unregister(websocket)
    
    
    
    async def start(self):
        """Start the WebSocket server."""
        self.logger.info(f'Starting WebSocket server on ws://{self.host}:{self.port}')
        async with websockets.serve(self.handler, self.host, self.port):
            await asyncio.Future()  # Run forever



def run_server(data_manager, config):
    """Helper to run server from config."""
    ws_config = config.get('websocket', {})
    server = WebSocketServer(
        data_manager,
        host=ws_config.get('host', 'localhost'),
        port=ws_config.get('port', 8765),
        rate=ws_config.get('update_rate_hz', 10)
    )
    asyncio.run(server.start())