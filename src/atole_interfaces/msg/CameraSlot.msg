# Una cámara conocida por el sistema.
string id                          # cam0 | cam1 | camC | cam2
string model                       # zedxhdr | zedxhdrmax | zedxm ...
string serial                      # "" si todavía no se ha leído
string role                        # eth | eih
string position                    # izquierda | derecha | centro | tcp
bool connected                     # detectada en el bus GMSL
bool active                        # su componente ZED está cargado
bool streaming
bool calibrated                    # hay matriz válida para su serial
bool display                       # la GUI muestra su imagen
bool cloud                         # Lichtblick muestra su nube de escena (gui_bridge)
