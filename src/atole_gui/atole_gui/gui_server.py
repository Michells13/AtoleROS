"""gui_server — sirve la web GUI (http://<ZED Box>:Gui/Port).

    /             web/ del paquete (index.html, app.js, rosbridge.js, style.css, layout de Lichtblick)
    /lichtblick/  build web estático de Lichtblick (Gui/LichtblickDir; release lichtblick-web.tar.gz)
Todo es local: la GUI funciona en campo sin internet. Los datos van por rosbridge (:9090, comandos y
estado) y foxglove_bridge (:8765, Lichtblick).
"""
import functools
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node


class Handler(SimpleHTTPRequestHandler):
    """Dos raíces: /lichtblick/… → Lichtblick; el resto → web/."""

    def __init__(self, *args, web=None, lichtblick=None, **kwargs):
        self.web, self.lichtblick = web, lichtblick
        super().__init__(*args, directory=str(web), **kwargs)

    def translate_path(self, path):
        clean = path.split('?', 1)[0].split('#', 1)[0]
        if clean == '/lichtblick':
            clean = '/lichtblick/'
        if clean.startswith('/lichtblick/') and self.lichtblick:
            rel = clean[len('/lichtblick/'):] or 'index.html'
            target = (self.lichtblick / rel).resolve()
            return str(target if str(target).startswith(str(self.lichtblick.resolve())) else self.lichtblick / 'index.html')
        return super().translate_path(path)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


class GuiServer(Node):

    def __init__(self):
        super().__init__('gui_server')
        self.server = None
        self.config = ConfigView(self, on_update=self._start)
        self.get_logger().info('gui_server: esperando /atole/config')

    def _start(self, cfg):
        if self.server is not None:
            return                      # el puerto y las rutas solo se leen al arrancar
        port = int(float(cfg.get('Gui/Port', 8080)))
        web = Path(get_package_share_directory('atole_gui')) / 'web'
        lichtblick = Path(cfg.get('Gui/LichtblickDir', '')).expanduser()
        if not (lichtblick / 'index.html').exists():
            self.get_logger().error(f'no está Lichtblick en {lichtblick}: la GUI funcionará sin el visor 3D '
                                    '(descarga lichtblick-web.tar.gz de su release y descomprímelo ahí)')
            lichtblick = None
        handler = functools.partial(Handler, web=web, lichtblick=lichtblick)
        self.server = ThreadingHTTPServer(('0.0.0.0', port), handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'web GUI en http://0.0.0.0:{port}/ (Lichtblick: {lichtblick or "no disponible"})')

    def destroy_node(self):
        if self.server is not None:
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    run_node(GuiServer, args)


if __name__ == '__main__':
    main()
