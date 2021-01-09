#include <SDL2/SDL.h>
#include <cuda.h>
#include <cmath>
#include "tracy/Tracy.hpp"

#define w 800
#define h 600
#define windowwidth 800
#define windowheight 600
__device__ void getCurrentValue(float* out, float λ, float dist) {
    float ret = 20*sin((2*M_PI)/λ*299792458*(dist/299792458));
    //printf("ret %f dist %f\n",ret,dist);
    *out = ret;
}


struct wavesource {
    float x;
    float y;
    float z;
    float λ;
};

struct point {
    float x;
    float y;
    float z;
    float value;
};

__global__ void parallelfunc(point* points, wavesource* sources, int max) {
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx < max) {
        for (int i = 0; i < max; i++) {
            float out = 0.0;
            getCurrentValue(&out,sources[idx].λ,sqrt(pow(sources[idx].x-points[i].x,2)+
                                                      pow(sources[idx].y-points[i].y,2)+
                                                      pow(sources[idx].z-points[i].z,2)));
            atomicAdd(&points[i].value,out);
        }
    }
}

int main() {
    SDL_Window* wnd = SDL_CreateWindow("Test frequency",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,
                                       windowwidth,windowheight,0);
    SDL_Renderer* rend = SDL_CreateRenderer(wnd,-1,SDL_RENDERER_ACCELERATED);

    float wave = 650.0*pow(10.0,-9.0);

    wavesource* sources;
    sources = (wavesource*)malloc(sizeof(wavesource)*w*h);

    point* points;
    point* origpoints;
    origpoints = (point*)malloc(sizeof(point)*w*h);
    points = (point*)malloc(sizeof(point)*w*h);

    for (int i = 0; i < w*h; i++) {
        sources[i] = wavesource{static_cast<float>(i/w),static_cast<float>(i%h),0,wave};
        origpoints[i] = point{static_cast<float>(i/w),static_cast<float>(i%h),100,0.0};
    }
    wavesource* dsources = nullptr;
    point* dpoints = nullptr;
    cudaMalloc((void**)&dsources, sizeof(wavesource)*w*h);
    cudaMalloc((void**)&dpoints, sizeof(point)*w*h);
    cudaMemcpy(dsources,sources,sizeof(wavesource)*w*h,cudaMemcpyHostToDevice);
    cudaMemcpy(dpoints,origpoints,sizeof(point)*w*h,cudaMemcpyHostToDevice);
    int threadsPerBlock = 256;
    int blocksPerGrid = (w*h + threadsPerBlock - 1) / threadsPerBlock;

    int width = windowwidth/w;
    int height = windowheight/h;
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

        {
            ZoneScopedN("Kernel running")
            parallelfunc<<<blocksPerGrid, threadsPerBlock>>>(dpoints, dsources, w * h);
            cudaDeviceSynchronize();
        }
        {
            ZoneScopedN("Memcpy results")
            cudaMemcpy(points, dpoints, sizeof(point) * w * h, cudaMemcpyDeviceToHost);
        }
        {
            ZoneScopedN("Reset GPU Memory")
            cudaMemcpy(dpoints, origpoints, sizeof(point) * w * h, cudaMemcpyHostToDevice);
        }
        {
            ZoneScopedN("Render")
            for (int i = 0; i < w*h; i++) {
                point _point = points[i];
                SDL_SetRenderDrawColor(rend, _point.value, _point.value, _point.value, 255);
                SDL_Rect rect = {static_cast<int>(_point.x * width),
                                 static_cast<int>(_point.y * height),
                                 width,
                                 height};
                SDL_RenderDrawRect(rend, &rect);
                SDL_RenderFillRect(rend, &rect);
            }
        FrameMark;
        }
        SDL_RenderPresent(rend);
        SDL_Delay(16);
    }
    cudaFree(dpoints);
    cudaFree(dsources);
    SDL_DestroyRenderer(rend);
    SDL_DestroyWindow(wnd);
    SDL_Quit();
    return 0;
}
