#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace sf;

const int ANCHO = 800;
const int ALTO = 600;
const int ESPACIADO_NODOS = 20;

struct Nodo {
    Vector2f posicion;
    bool es_obstaculo = false;
    int indice;
};

struct Arista {
    int destino;
    float costo;
    Arista(int d, float c) : destino(d), costo(c) {}
};

struct Estado {
    int nodo;
    float costo;
    Estado(int n, float c) : nodo(n), costo(c) {}
    bool operator<(const Estado& otro) const {
        return costo > otro.costo;
    }
};

int obtenerIndice(int x, int y, int columnas) {
    return y * columnas + x;
}

int main() {
    RenderWindow ventana(VideoMode(ANCHO, ALTO), "Visualizacion de Dijkstra");

    int columnas = ANCHO / ESPACIADO_NODOS;
    int filas = ALTO / ESPACIADO_NODOS;
    int totalNodos = columnas * filas;  

    vector<Nodo> nodos(totalNodos);
    for (int y = 0; y < filas; y++) {
        for (int x = 0; x < columnas; x++) {
            int idx = obtenerIndice(x, y, columnas);
            nodos[idx].posicion = Vector2f(x * ESPACIADO_NODOS + ESPACIADO_NODOS / 2.f, y * ESPACIADO_NODOS + ESPACIADO_NODOS / 2.f);
            nodos[idx].indice = idx;
        }
    }

    vector<vector<Arista>> grafo(totalNodos);
    auto esValido = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < columnas && y < filas;
    };

    for (int y = 0; y < filas; y++) {
        for (int x = 0; x < columnas; x++) {
            int desde = obtenerIndice(x, y, columnas);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (!(dx == 0 && dy == 0)) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (esValido(nx, ny)) {
                            int hacia = obtenerIndice(nx, ny, columnas);
                            float distancia = sqrt(dx * dx + dy * dy) * ESPACIADO_NODOS;
                            grafo[desde].push_back(Arista(hacia, distancia));
                        }
                    }
                }
            }
        }
    }

    CircleShape agente(8);
    agente.setFillColor(Color::Blue);
    int nodoInicio = obtenerIndice(5, 5, columnas);
    Vector2f posicionAgente = nodos[nodoInicio].posicion;

    vector<int> camino;
    size_t indiceCamino = 0;
    vector<int> nodosVisitados;

    auto dijkstra = [&](int inicio, int meta) -> vector<int> {
        nodosVisitados.clear();
        vector<float> distancias(totalNodos, 1e9f);
        vector<int> desde(totalNodos, -1);
        priority_queue<Estado> cola;

        distancias[inicio] = 0;
        cola.push(Estado(inicio, 0));

        while (!cola.empty()) {
            Estado actual = cola.top();
            cola.pop();

            if (actual.nodo == meta) {
                break;
            }

            if (actual.costo <= distancias[actual.nodo]) {
                nodosVisitados.push_back(actual.nodo);

                for (auto& arista : grafo[actual.nodo]) {
                    int siguiente = arista.destino;
                    if (!nodos[siguiente].es_obstaculo) {
                        float nuevoCosto = distancias[actual.nodo] + arista.costo;
                        if (nuevoCosto < distancias[siguiente]) {
                            distancias[siguiente] = nuevoCosto;
                            desde[siguiente] = actual.nodo;
                            cola.push(Estado(siguiente, nuevoCosto));
                        }
                    }
                }
            }
        }

        vector<int> caminoReverso;
        if (desde[meta] == -1) return {};
        for (int actual = meta; actual != -1; actual = desde[actual]) {
            caminoReverso.push_back(actual);
        }
        reverse(caminoReverso.begin(), caminoReverso.end());
        return caminoReverso;
    };

    while (ventana.isOpen()) {
        Event evento;
        while (ventana.pollEvent(evento)) {
            if (evento.type == Event::Closed)
                ventana.close();

            if (evento.type == Event::MouseButtonPressed) {
                int mx = evento.mouseButton.x;
                int my = evento.mouseButton.y;
                int gx = mx / ESPACIADO_NODOS;
                int gy = my / ESPACIADO_NODOS;
                if (esValido(gx, gy)) {
                    int nodoClickeado = obtenerIndice(gx, gy, columnas);
                    if (evento.mouseButton.button == Mouse::Left) {
                        nodos[nodoClickeado].es_obstaculo = !nodos[nodoClickeado].es_obstaculo;
                    } else if (evento.mouseButton.button == Mouse::Right) {
                        int nodoAgenteActual = obtenerIndice((int)(posicionAgente.x / ESPACIADO_NODOS), (int)(posicionAgente.y / ESPACIADO_NODOS), columnas);
                        camino = dijkstra(nodoAgenteActual, nodoClickeado);
                        indiceCamino = 0;
                    }
                }
            }
        }

        if (indiceCamino < camino.size()) {
            Vector2f destino = nodos[camino[indiceCamino]].posicion;
            Vector2f direccion = destino - posicionAgente;
            float longitud = sqrt(direccion.x * direccion.x + direccion.y * direccion.y);
            if (longitud > 1.0f) {
                direccion /= longitud;
                posicionAgente += direccion * 2.5f;
            } else {
                posicionAgente = destino;
                indiceCamino++;
            }
        }

        ventana.clear();

        for (auto& nodo : nodos) {
            RectangleShape rectangulo(Vector2f(ESPACIADO_NODOS - 1, ESPACIADO_NODOS - 1));
            rectangulo.setOrigin(ESPACIADO_NODOS / 2.f, ESPACIADO_NODOS / 2.f);
            rectangulo.setPosition(nodo.posicion);

            if (nodo.es_obstaculo)
                rectangulo.setFillColor(Color::Red);
            else
                rectangulo.setFillColor(Color(70, 70, 70));

            ventana.draw(rectangulo);
        }

        for (int idx : nodosVisitados) {
            RectangleShape rectangulo(Vector2f(ESPACIADO_NODOS - 1, ESPACIADO_NODOS - 1));
            rectangulo.setOrigin(ESPACIADO_NODOS / 2.f, ESPACIADO_NODOS / 2.f);
            rectangulo.setPosition(nodos[idx].posicion);
            rectangulo.setFillColor(Color(255, 140, 0, 100));
            ventana.draw(rectangulo);
        }

        for (int i = 0; i + 1 < camino.size(); i++) {
            Vertex linea[] = {
                Vertex(nodos[camino[i]].posicion, Color::Green),
                Vertex(nodos[camino[i + 1]].posicion, Color::Green)
            };
            ventana.draw(linea, 2, Lines);
        }

        agente.setPosition(posicionAgente - Vector2f(agente.getRadius(), agente.getRadius()));
        ventana.draw(agente);

        ventana.display();
    }

    return 0;
}
