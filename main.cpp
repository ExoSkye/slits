#include <cmath>
#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "tracy/TracyC.h"
#include "tracy/Tracy.hpp"
#include <cuda.h>

#define w 600
#define h 800

long double getCurrentValue(long double A, long double λ, long double ϕ, long double x, long double v, long double t) {
    long double ret = A*std::sin(((2*M_PIl)/λ)*(x+v*t)+ϕ);
    return ret;
}

typedef glm::vec<2,long double,glm::defaultp> vec2;
typedef glm::vec<3,long double,glm::defaultp> vec3;

struct wavesource {
    vec3 location;
    long double λ;
    long double A;
};

int main() {
    SDL_Window* wnd = SDL_CreateWindow("Test frequency",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,800,600,0);
    SDL_Renderer* rend = SDL_CreateRenderer(wnd,-1,SDL_RENDERER_ACCELERATED);

    double wave = 650.0*pow(10.0,-9.0);

    std::vector<std::vector<wavesource>> sources;

    for (int x = 0; x < w; x++) {
        sources.emplace_back(std::vector<wavesource>{});
        for (int y = 0; y < h; y++) {
            sources[x].emplace_back(wavesource{vec3{x + 1, y + 1, 1}, 650 * pow(10, -9), 20});
        }
    }

    long double points[w][h];

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
            }
        }
        SDL_SetRenderDrawColor(rend,0,0,0,255);
        SDL_RenderClear(rend);
        SDL_SetRenderDrawColor(rend,255,255,255,255);
        int width = 600/w;
        int height = 800/h;
        TracyCZoneN(mainloop,"Main loop",true)
        for (int x1 = 0; x1 < w; x1++) {
            ZoneScopedN("X LOOP")
            for (int y1 = 0; y1 < h; y1++) {
                ZoneScopedN("Y LOOP")
                points[x1][y1] = 0;
                std::mutex point;
                for (int x2 = 0; x2 < w; x2++) {
                    for (int y2 = 0; y2 < h; y2++) {
                        point.lock();
                        points[x1][y1] += getCurrentValue(20, sources[x2][y2].λ, 0, 0, 299792458,
                                                          glm::distance(sources[x2][y2].location, vec3{x1, y1, 100}) /
                                                          299792458);
                        point.unlock();
                    }

                }
                SDL_SetRenderDrawColor(rend, std::min(points[x1][y1], 255.0l), 0, 0, 255);
                SDL_RenderDrawPoint(rend, x1,y1);
            }
        }
        TracyCZoneEnd(mainloop)
        SDL_RenderPresent(rend);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(rend);
    SDL_DestroyWindow(wnd);
    SDL_Quit();
    return 0;
}
