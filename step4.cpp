/*
  CardCraft
  A simple voxel engine for M5Cardputer
*/
/**
=============== PLATFORMIO.INI ===============
[env:m5stack-stamps3]
platform = espressif32@6.4.0
board = m5stack-stamps3
framework = arduino
lib_deps = m5stack/M5Cardputer@^1.0.2
build_flags = -DCORE_DEBUG_LEVEL=0 -DARDUINO_USB_CDC_ON_BOOT=1 -O3 -ffast-math -funroll-loops -DBOARD_HAS_PSRAM=0 -D SCK=40 -D MISO=39 -D MOSI=14 -D SS=12
board_build.partitions = huge_app.csv
monitor_speed = 115200
upload_speed = 1500000
*/
#include <M5Cardputer.h>
// ============== CONFIG.H ==============
#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
constexpr int SCREEN_W = 240, SCREEN_H = 135, SCREEN_CENTER_X = SCREEN_W / 2, SCREEN_CENTER_Y = SCREEN_H / 2;
constexpr int CHUNK_SIZE_X = 8, CHUNK_SIZE_Y = 48, CHUNK_SIZE_Z = 8, CHUNK_SIZE_XZ = CHUNK_SIZE_X, CHUNK_POOL_SIZE = 25, LOADED_RADIUS = 2, WORLD_HEIGHT = CHUNK_SIZE_Y, SEA_LEVEL = 16, BASE_HEIGHT = 22, MIN_HEIGHT = 6, MAX_HEIGHT = 42;
constexpr int MAX_RENDER_DIST = 32, MAX_RAY_STEPS = 34;
constexpr float GRAVITY_VAL = 0.025f, JUMP_VELOCITY = 0.35f, MOVE_SPEED = 0.18f, TURN_SPEED = 0.08f, LOOK_SPEED = 0.06f, PLAYER_WIDTH = 0.6f, PLAYER_HEIGHT = 1.7f, EYE_HEIGHT = 1.5f;
enum BlockType : uint8_t { BLOCK_AIR = 0, BLOCK_STONE = 1, BLOCK_DIRT = 2, BLOCK_GRASS = 3, BLOCK_WOOD = 4, BLOCK_LEAVES = 5, BLOCK_BEDROCK = 6, BLOCK_GRAVEL = 7, BLOCK_COUNT = 8 };
constexpr uint16_t COLOR_SKY = 0x6DBF, COLOR_STONE_TOP = 0x8410, COLOR_DIRT_TOP = 0x8A22, COLOR_GRASS_TOP = 0x3666, COLOR_WOOD_TOP = 0x6180, COLOR_LEAVES_TOP = 0x2D83, COLOR_BEDROCK_TOP = 0x2104, COLOR_GRAVEL_TOP = 0x9492, COLOR_STONE_SIDE = 0x630C, COLOR_DIRT_SIDE = 0x6180, COLOR_GRASS_SIDE = 0x6180, COLOR_WOOD_SIDE = 0x5140, COLOR_LEAVES_SIDE = 0x2562, COLOR_BEDROCK_SIDE = 0x18C3, COLOR_GRAVEL_SIDE = 0x7BCF;
constexpr int MAX_MODIFICATIONS = 256, SD_CS_PIN = 12, SD_MOSI_PIN = 14, SD_MISO_PIN = 39, SD_CLK_PIN = 40;
constexpr float MY_PI = 3.14159265f, FOV = 1.2f;
#endif
//============== TYPES.H ==============
#ifndef TYPES_H
#define TYPES_H
#include <math.h>
#include <string.h>

static inline float fastInvSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float xhalf = 0.5f * x;
    uint32_t i;
    memcpy(&i, &x, 4);
    i = 0x5f3759df - (i >> 1);
    float y;
    memcpy(&y, &i, 4);
    y = y * (1.5f - xhalf * y * y);  // 1 次牛顿迭代
    return y;
}
struct Vec3f {
    float x, y, z;
    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    Vec3f operator+(const Vec3f &o) const { return Vec3f(x + o.x, y + o.y, z + o.z); }
    Vec3f operator-(const Vec3f &o) const { return Vec3f(x - o.x, y - o.y, z - o.z); }
    Vec3f operator*(float s) const { return Vec3f(x * s, y * s, z * s); }
    float length() const { return sqrtf(x * x + y * y + z * z); }
    Vec3f normalized() const { float len = length(); return len < 0.0001f ? Vec3f(0, 0, 0) : Vec3f(x / len, y / len, z / len); }
};
struct Vec3i {
    int x, y, z;
    Vec3i() : x(0), y(0), z(0) {}
    Vec3i(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
    bool operator==(const Vec3i &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct RayHit {
    bool hit; float distance; Vec3i blockPos, normal; BlockType block;
    RayHit() : hit(false), distance(0), block(BLOCK_AIR) {}
};
struct BlockMod { int16_t x, y, z; uint8_t blockType; bool valid; };
inline int fastFloor(float x) { int xi = (int)x; return x < xi ? xi - 1 : xi; }
inline float frac(float x) { return x - fastFloor(x); }
inline float clampf(float x, float minVal, float maxVal) { return x < minVal ? minVal : (x > maxVal ? maxVal : x); }
inline int clampi(int x, int minVal, int maxVal) { return x < minVal ? minVal : (x > maxVal ? maxVal : x); }
class FastMath {
public:
    static constexpr int TABLE_SIZE = 256;
    float sinTable[TABLE_SIZE], cosTable[TABLE_SIZE];
    void init() { for (int i = 0; i < TABLE_SIZE; i++) { float angle = (float)i / TABLE_SIZE * 2.0f * MY_PI; sinTable[i] = sinf(angle); cosTable[i] = cosf(angle); } }
    float fastSin(float angle) const {
    // 无 while：直接利用 uint32_t 溢出 + mask 实现角度环绕
    const float k = (float)TABLE_SIZE / (2.0f * MY_PI);
    uint32_t idx = (uint32_t)(angle * k);
    idx &= (TABLE_SIZE - 1);
    return sinTable[idx];
}
float fastCos(float angle) const {
    const float k = (float)TABLE_SIZE / (2.0f * MY_PI);
    uint32_t idx = (uint32_t)(angle * k);
    idx &= (TABLE_SIZE - 1);
    return cosTable[idx];
}
};
extern FastMath g_fastMath;
#endif
// ============== NOISE.H ==============
#ifndef NOISE_H
#define NOISE_H
class PerlinNoise {
private:
    uint8_t perm[512];
    uint32_t seed;
    static constexpr int8_t grad2[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
    static constexpr int8_t grad3[12][3] = {{1,1,0},{-1,1,0},{1,-1,0},{-1,-1,0},{1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},{0,1,1},{0,-1,1},{0,1,-1},{0,-1,-1}};
    inline float fade(float t) const { return t*t*t*(t*(t*6-15)+10); }
    inline float lerp(float a, float b, float t) const { return a+t*(b-a); }
    inline float grad2d(int hash, float x, float y) const { int h=hash&7; return grad2[h][0]*x+grad2[h][1]*y; }
    inline float grad3d(int hash, float x, float y, float z) const { int h=hash%12; return grad3[h][0]*x+grad3[h][1]*y+grad3[h][2]*z; }
public:
    void init(uint32_t s) {
        seed=s; for(int i=0;i<256;i++) perm[i]=i;
        uint32_t rng=seed;
        for(int i=255;i>0;i--) { rng=rng*1103515245+12345; int j=(rng>>16)%(i+1); uint8_t tmp=perm[i]; perm[i]=perm[j]; perm[j]=tmp; }
        for(int i=0;i<256;i++) perm[256+i]=perm[i];
    }
    float noise2D(float x, float y) const {
        int X=fastFloor(x)&255, Y=fastFloor(y)&255;
        x-=fastFloor(x); y-=fastFloor(y);
        float u=fade(x), v=fade(y);
        int A=perm[X]+Y, B=perm[X+1]+Y;
        float g00=grad2d(perm[A],x,y), g10=grad2d(perm[B],x-1,y), g01=grad2d(perm[A+1],x,y-1), g11=grad2d(perm[B+1],x-1,y-1);
        return lerp(lerp(g00,g10,u),lerp(g01,g11,u),v);
    }
    float noise3D(float x, float y, float z) const {
        int X=fastFloor(x)&255, Y=fastFloor(y)&255, Z=fastFloor(z)&255;
        x-=fastFloor(x); y-=fastFloor(y); z-=fastFloor(z);
        float u=fade(x), v=fade(y), w=fade(z);
        int A=perm[X]+Y, AA=perm[A]+Z, AB=perm[A+1]+Z, B=perm[X+1]+Y, BA=perm[B]+Z, BB=perm[B+1]+Z;
        float g000=grad3d(perm[AA],x,y,z), g100=grad3d(perm[BA],x-1,y,z), g010=grad3d(perm[AB],x,y-1,z), g110=grad3d(perm[BB],x-1,y-1,z);
        float g001=grad3d(perm[AA+1],x,y,z-1), g101=grad3d(perm[BA+1],x-1,y,z-1), g011=grad3d(perm[AB+1],x,y-1,z-1), g111=grad3d(perm[BB+1],x-1,y-1,z-1);
        float lerpX0=lerp(g000,g100,u), lerpX1=lerp(g010,g110,u), lerpX2=lerp(g001,g101,u), lerpX3=lerp(g011,g111,u);
        float lerpY0=lerp(lerpX0,lerpX1,v), lerpY1=lerp(lerpX2,lerpX3,v);
        return lerp(lerpY0,lerpY1,w);
    }
    float fractalNoise2D(float x, float y, int octaves, float persistence) const {
        float total=0, freq=1, amp=1, maxVal=0;
        for(int i=0;i<octaves;i++) { total+=noise2D(x*freq,y*freq)*amp; maxVal+=amp; amp*=persistence; freq*=2; }
        return total/maxVal;
    }
    float fractalNoise3D(float x, float y, float z, int octaves, float persistence) const {
        float total=0, freq=1, amp=1, maxVal=0;
        for(int i=0;i<octaves;i++) { total+=noise3D(x*freq,y*freq,z*freq)*amp; maxVal+=amp; amp*=persistence; freq*=2; }
        return total/maxVal;
    }
    uint32_t getSeed() const { return seed; }
};
extern PerlinNoise g_noise;
#endif
// ============== CHUNK.H ==============
#ifndef CHUNK_H
#define CHUNK_H
#include <string.h>
struct Chunk {
    
    int16_t cx, cz;
    uint8_t blocks[CHUNK_SIZE_X][CHUNK_SIZE_Y][CHUNK_SIZE_Z];
    bool loaded, modified;
    void clear() { loaded = false; modified = false; cx = 0; cz = 0; }
    BlockType getBlock(int lx, int ly, int lz) const { if (lx < 0 || lx >= CHUNK_SIZE_X || ly < 0 || ly >= CHUNK_SIZE_Y || lz < 0 || lz >= CHUNK_SIZE_Z) return BLOCK_AIR; return (BlockType)blocks[lx][ly][lz]; }
    void setBlock(int lx, int ly, int lz, BlockType type) { if (lx >= 0 && lx < CHUNK_SIZE_X && ly >= 0 && ly < CHUNK_SIZE_Y && lz >= 0 && lz < CHUNK_SIZE_Z) { blocks[lx][ly][lz] = type; modified = true; } }
    inline uint32_t hash3(int x, int y, int z, uint32_t seed) const { uint32_t h = seed; h ^= x * 374761393u; h ^= y * 668265263u; h ^= z * 1274126177u; h = (h ^ (h >> 13)) * 1103515245u; return h; }
    void generate(const PerlinNoise &noise, uint32_t worldSeed)
    {
        int worldX = cx * CHUNK_SIZE_X, worldZ = cz * CHUNK_SIZE_Z;
        uint32_t rng = worldSeed ^ (cx * 73856093) ^ (cz * 19349663);
        struct TreeReq { uint8_t lx, lz; uint8_t ly; uint32_t rng; };
        TreeReq treeReq[32];
        uint8_t treeCount = 0;
        
        for (int lx = 0; lx < CHUNK_SIZE_X; lx++)
        {
            for (int lz = 0; lz < CHUNK_SIZE_Z; lz++)
            {
                int wx = worldX + lx, wz = worldZ + lz;
                float baseNoise = noise.fractalNoise2D(wx * 0.01f, wz * 0.01f, 3, 0.5f);
                float mountainNoise = noise.noise2D(wx * 0.02f + 100, wz * 0.02f + 100);
                mountainNoise = (mountainNoise > 0) ? mountainNoise * mountainNoise * 15.0f : 0;
                int height = clampi(BASE_HEIGHT + (int)(baseNoise * 10.0f + mountainNoise), MIN_HEIGHT, MAX_HEIGHT);
                for (int ly = 0; ly < CHUNK_SIZE_Y; ly++)
                {
                    BlockType type = BLOCK_AIR;
                    if (ly == 0) type = BLOCK_BEDROCK;
                    else if (ly < 3) type = ((hash3(wx, ly, wz, worldSeed) & 1) == 0) ? BLOCK_BEDROCK : BLOCK_STONE;
                    else if (ly < height - 4)
                    {
                        type = BLOCK_STONE;
                        if (ly > 4 && ly < height - 6 && ((hash3(wx >> 1, ly >> 1, wz >> 1, worldSeed + 12345) & 0x1F) == 0)) type = BLOCK_AIR;
                        if (type == BLOCK_STONE && ly < 10 && ((hash3(wx, ly, wz, worldSeed + 99999) & 0x7) == 0)) type = BLOCK_GRAVEL;
                    }
                    else if (ly < height) type = BLOCK_DIRT;
                    else if (ly == height) type = BLOCK_GRASS;
                    blocks[lx][ly][lz] = type;
                }
                rng = rng * 1103515245 + 12345;
                if (height > SEA_LEVEL && height < MAX_HEIGHT - 7 && (rng & 0x7F) < 3) {
                if (lx >= 2 && lx < CHUNK_SIZE_X - 2 && lz >= 2 && lz < CHUNK_SIZE_Z - 2) {
                if (treeCount < 32) treeReq[treeCount++] = {(uint8_t)lx, (uint8_t)lz, (uint8_t)(height + 1), rng};
                    }
                }
            }
        }
        for (uint8_t i = 0; i < treeCount; i++) {
    generateTree(treeReq[i].lx, treeReq[i].ly, treeReq[i].lz, treeReq[i].rng);
}
        loaded = true; modified = false;
    }
    void generateTree(int lx, int ly, int lz, uint32_t rng)
    {
        int trunkHeight = 4 + ((rng >> 8) % 3);
        for (int i = 0; i < trunkHeight && ly + i < CHUNK_SIZE_Y; i++) blocks[lx][ly + i][lz] = BLOCK_WOOD;
        int leafStart = ly + trunkHeight - 3, leafEnd = ly + trunkHeight;
        for (int ty = leafStart; ty <= leafEnd; ty++)
        {
            if (ty < 0 || ty >= CHUNK_SIZE_Y) continue;
            int distFromTop = leafEnd - ty, radius = (distFromTop <= 1) ? 1 : 2;
            for (int dx = -radius; dx <= radius; dx++)
            {
                for (int dz = -radius; dz <= radius; dz++)
                {
                    int tx = lx + dx, tz = lz + dz;
                    if (tx < 0 || tx >= CHUNK_SIZE_X || tz < 0 || tz >= CHUNK_SIZE_Z) continue;
                    if (radius == 2 && abs(dx) == 2 && abs(dz) == 2) continue;
                    if (blocks[tx][ty][tz] == BLOCK_AIR) blocks[tx][ty][tz] = BLOCK_LEAVES;
                }
            }
        }
    }
};
#endif
// ============== STORAGE.H ==============
#ifndef STORAGE_H
#define STORAGE_H
#include <FS.h>
#include <SD.h>
#include <SPI.h>
class Storage {
private:
    bool sdAvailable;
    char worldPath[64];
    int currentSlot;
public:
    void init() {
        SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
        sdAvailable = SD.begin(SD_CS_PIN, SPI, 20000000);
        if (sdAvailable && !SD.exists("/microcraft")) SD.mkdir("/microcraft");
        currentSlot = 1;
        snprintf(worldPath, sizeof(worldPath), "/microcraft/world_1");
    }
    bool isAvailable() const { return sdAvailable; }
    void setWorldSlot(int slot) {
        currentSlot = slot;
        snprintf(worldPath, sizeof(worldPath), "/microcraft/world_%d", slot);
        if (sdAvailable && !SD.exists(worldPath)) SD.mkdir(worldPath);
    }
    bool hasSave(int slot) {
        if (!sdAvailable) return false;
        char path[64];
        snprintf(path, sizeof(path), "/microcraft/world_%d/seed.dat", slot);
        return SD.exists(path);
    }
    void saveWorldSeed(uint32_t seed) {
        if (!sdAvailable) return;
        char path[80];
        snprintf(path, sizeof(path), "%s/seed.dat", worldPath);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f) { f.write((uint8_t*)&seed, sizeof(uint32_t)); f.close(); }
    }
    uint32_t loadWorldSeed() {
        if (!sdAvailable) return 0;
        char path[80];
        snprintf(path, sizeof(path), "%s/seed.dat", worldPath);
        if (!SD.exists(path)) return 0;
        File f = SD.open(path, FILE_READ);
        uint32_t seed = 0;
        if (f) { f.read((uint8_t*)&seed, sizeof(uint32_t)); f.close(); }
        return seed;
    }
    void saveChunk(Chunk *chunk) {
        if (!sdAvailable || !chunk->modified) return;
        char path[80];
        snprintf(path, sizeof(path), "%s/c_%d_%d.chk", worldPath, chunk->cx, chunk->cz);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f) {
            f.write((uint8_t*)&chunk->cx, 2); f.write((uint8_t*)&chunk->cz, 2);
            f.write((uint8_t*)chunk->blocks, sizeof(chunk->blocks));
            f.close(); chunk->modified = false;
        }
    }
    bool loadChunk(Chunk *chunk) {
        if (!sdAvailable) return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/c_%d_%d.chk", worldPath, chunk->cx, chunk->cz);
        if (!SD.exists(path)) return false;
        File f = SD.open(path, FILE_READ);
        if (!f) return false;
        int16_t cx, cz;
        f.read((uint8_t*)&cx, 2); f.read((uint8_t*)&cz, 2);
        if (cx != chunk->cx || cz != chunk->cz) { f.close(); return false; }
        f.read((uint8_t*)chunk->blocks, sizeof(chunk->blocks));
        f.close(); chunk->loaded = true; chunk->modified = false;
        return true;
    }
    bool writeData(const char *filename, const uint8_t *data, size_t len) {
        if (!sdAvailable) return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f) { f.write(data, len); f.close(); return true; }
        return false;
    }
    bool readData(const char *filename, uint8_t *buffer, size_t len) {
        if (!sdAvailable) return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        if (!SD.exists(path)) return false;
        File f = SD.open(path, FILE_READ);
        if (f) { f.read(buffer, len); f.close(); return true; }
        return false;
    }
};
extern Storage g_storage;
#endif
// ============== WORLD.H ==============
// ============== WORLD.H ==============
#ifndef WORLD_H
#define WORLD_H
#include <Arduino.h>

extern volatile uint32_t g_prof_chunkSaveMaxUs;
extern volatile uint32_t g_prof_chunkLoadMaxUs;
extern volatile uint32_t g_prof_chunkGenMaxUs;

class World {
public:
    static constexpr uint8_t MAX_FALLING = 12;
    struct FallingEnt { float x, y, z; float vy; BlockType type; bool active; };
    const FallingEnt *getFallingEnts() const { return falling; }
    static constexpr uint8_t getMaxFallingEnts() { return MAX_FALLING; }
private:
    Chunk chunks[CHUNK_POOL_SIZE];
    uint32_t worldSeed;
    int centerCX, centerCZ;
    bool centerValid;
    // 新增：已加载区块指针网格（固定 5x5，对应 LOADED_RADIUS=2）
    Chunk *grid[5][5];

    FallingEnt falling[MAX_FALLING];

    int getChunkIndex(int cx, int cz) const {
        int wrapped_cx = ((cx % 5) + 5) % 5; int wrapped_cz = ((cz % 5) + 5) % 5;
        return wrapped_cz * 5 + wrapped_cx;
    }

    inline bool isFallingBlock(BlockType t) const { return t == BLOCK_GRAVEL; }

    void setBlockRaw(int wx, int wy, int wz, BlockType type) {
        if (wy < 0 || wy >= CHUNK_SIZE_Y) return;
        int cx = worldToChunkCoord(wx);
        int cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx);
        int lz = worldToLocalCoord(wz);
        Chunk *c = ensureChunk(cx, cz);
        c->setBlock(lx, wy, lz, type);
    }

    bool spawnFalling(int wx, int wy, int wz, BlockType type) {
        for (int i = 0; i < MAX_FALLING; i++) {
            if (!falling[i].active) {
                setBlockRaw(wx, wy, wz, BLOCK_AIR);
                falling[i] = {(float)wx + 0.5f, (float)wy + 0.5f, (float)wz + 0.5f, 0.0f, type, true};
                return true;
            }
        }
        return false;
    }

public:
    void init(uint32_t seed) {
        worldSeed = seed; centerCX = 0; centerCZ = 0; centerValid = false;
        g_noise.init(seed);
        for (int i = 0; i < CHUNK_POOL_SIZE; i++) chunks[i].clear();
        for (int i = 0; i < MAX_FALLING; i++) falling[i].active = false;
        for (int gz = 0; gz < 5; gz++)
            for (int gx = 0; gx < 5; gx++)
                grid[gx][gz] = nullptr;
    }

    uint32_t getSeed() const { return worldSeed; }

    static int worldToChunkCoord(int w) { return w >= 0 ? w / CHUNK_SIZE_XZ : (w - CHUNK_SIZE_XZ + 1) / CHUNK_SIZE_XZ; }
    static int worldToLocalCoord(int w) { int local = w % CHUNK_SIZE_XZ; return local >= 0 ? local : local + CHUNK_SIZE_XZ; }

        Chunk *ensureChunk(int cx, int cz) {
        int idx = getChunkIndex(cx, cz);
        Chunk *chunk = &chunks[idx];
        if (!chunk->loaded || chunk->cx != cx || chunk->cz != cz) {
            // Save logic with profiling
            if (chunk->loaded && chunk->modified) {
                uint32_t t0 = micros();
                g_storage.saveChunk(chunk);
                uint32_t dt = micros() - t0;
                if (dt > g_prof_chunkSaveMaxUs) g_prof_chunkSaveMaxUs = dt;
            }

            chunk->clear();
            chunk->cx = cx;
            chunk->cz = cz;

            // Load logic with profiling
            uint32_t t1 = micros();
            bool loaded = g_storage.loadChunk(chunk);
            uint32_t dtLoad = micros() - t1;
            if (dtLoad > g_prof_chunkLoadMaxUs) g_prof_chunkLoadMaxUs = dtLoad;

            // Generate logic with profiling
            if (!loaded) {
                uint32_t t2 = micros();
                chunk->generate(g_noise, worldSeed);
                uint32_t dtGen = micros() - t2;
                if (dtGen > g_prof_chunkGenMaxUs) g_prof_chunkGenMaxUs = dtGen;
            }
        }
        return chunk;
    }

    void saveAllChunks() {
        for (int i = 0; i < CHUNK_POOL_SIZE; i++)
            if (chunks[i].loaded && chunks[i].modified) g_storage.saveChunk(&chunks[i]);
    }

    BlockType getBlock(int wx, int wy, int wz) {
        if (wy < 0 || wy >= CHUNK_SIZE_Y) return BLOCK_AIR;
        int cx = worldToChunkCoord(wx), cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx), lz = worldToLocalCoord(wz);
        return ensureChunk(cx, cz)->getBlock(lx, wy, lz);
    }

    BlockType getBlockFast(int wx, int wy, int wz) {
        if (wy < 0 || wy >= CHUNK_SIZE_Y) return BLOCK_AIR;
        int cx = wx >> 3, cz = wz >> 3;
        int lx = wx & 7,  lz = wz & 7;
        int idx = getChunkIndex(cx, cz);
        Chunk *chunk = &chunks[idx];
        if (!chunk->loaded || chunk->cx != cx || chunk->cz != cz) return BLOCK_AIR;
        return chunk->getBlock(lx, wy, lz);
    }

    void setBlock(int wx, int wy, int wz, BlockType type) {
        if (wy < 0 || wy >= CHUNK_SIZE_Y) return;
        if (isFallingBlock(type) && getBlock(wx, wy - 1, wz) == BLOCK_AIR && wy > 0) {
            spawnFalling(wx, wy, wz, type);
            return;
        }
        int cx = worldToChunkCoord(wx), cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx), lz = worldToLocalCoord(wz);
        Chunk *chunk = ensureChunk(cx, cz);
        BlockType old = chunk->getBlock(lx, wy, lz);
        if (old == type) return;
        chunk->setBlock(lx, wy, lz, type);
        if (type == BLOCK_AIR) {
            for (int y = wy + 1; y >= 0 && y < CHUNK_SIZE_Y; y++) {
                BlockType b = getBlock(wx, y, wz);
                BlockType under = getBlock(wx, y - 1, wz);
                if (b == BLOCK_GRAVEL && under == BLOCK_AIR) {
                    if (!spawnFalling(wx, y, wz, b)) break;
                    continue;
                }
                break;
            }
        }
    }

    void updateAroundPlayer(float px, float pz) {
    int newCX = worldToChunkCoord(fastFloor(px));
    int newCZ = worldToChunkCoord(fastFloor(pz));
    if (centerValid && newCX == centerCX && newCZ == centerCZ) return;

    for (int dx = -LOADED_RADIUS; dx <= LOADED_RADIUS; dx++) {
        for (int dz = -LOADED_RADIUS; dz <= LOADED_RADIUS; dz++) {
            Chunk *c = ensureChunk(newCX + dx, newCZ + dz);
            grid[dx + LOADED_RADIUS][dz + LOADED_RADIUS] = c;
        }
    }
    centerCX = newCX; centerCZ = newCZ; centerValid = true;
    }

    RayHit raycast(Vec3f origin, Vec3f direction, float maxDist) {
        RayHit result; result.hit = false;
        float len2 = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (len2 < 1e-12f) return result;
        if (fabsf(len2 - 1.0f) > 1e-2f) { // 阈值放宽，避免因 fastInvSqrt 误差频繁触发
            float invLen = fastInvSqrt(len2);
            direction.x *= invLen; direction.y *= invLen; direction.z *= invLen;
        }
        int mapX = fastFloor(origin.x), mapY = fastFloor(origin.y), mapZ = fastFloor(origin.z);
        float deltaDistX = (direction.x == 0) ? 1e30f : fabsf(1.0f / direction.x);
        float deltaDistY = (direction.y == 0) ? 1e30f : fabsf(1.0f / direction.y);
        float deltaDistZ = (direction.z == 0) ? 1e30f : fabsf(1.0f / direction.z);
        int stepX = (direction.x < 0) ? -1 : 1;
        int stepY = (direction.y < 0) ? -1 : 1;
        int stepZ = (direction.z < 0) ? -1 : 1;
        float sideDistX = (direction.x < 0) ? (origin.x - mapX) * deltaDistX : (mapX + 1.0f - origin.x) * deltaDistX;
        float sideDistY = (direction.y < 0) ? (origin.y - mapY) * deltaDistY : (mapY + 1.0f - origin.y) * deltaDistY;
        float sideDistZ = (direction.z < 0) ? (origin.z - mapZ) * deltaDistZ : (mapZ + 1.0f - origin.z) * deltaDistZ;
        int side = 0;
        for (int i = 0; i < MAX_RAY_STEPS; i++) {
            if (sideDistX < sideDistY && sideDistX < sideDistZ) { sideDistX += deltaDistX; mapX += stepX; side = 0; }
            else if (sideDistY < sideDistZ) { sideDistY += deltaDistY; mapY += stepY; side = 1; }
            else { sideDistZ += deltaDistZ; mapZ += stepZ; side = 2; }
            if (mapY < 0 || mapY >= CHUNK_SIZE_Y) continue;
            float dist = (side == 0) ? sideDistX - deltaDistX : (side == 1) ? sideDistY - deltaDistY : sideDistZ - deltaDistZ;
            if (dist > maxDist) break;
            // 仅在加载半径内才取块，否则当 AIR（和你原来的 getBlockFast 行为一致）
if (!centerValid) continue;

const int R = LOADED_RADIUS; // 2
static_assert(LOADED_RADIUS == 2, "grid[5][5] assumes LOADED_RADIUS==2");

// 缓存当前 chunk，避免每步都算 dx/dz（chunk 8 格才变一次）
int curCX = 0x7fffffff;
int curCZ = 0x7fffffff;
static Chunk *curChunk = nullptr;

int cx = mapX >> 3;
int cz = mapZ >> 3;
if (cx != curCX || cz != curCZ) {
    curCX = cx; curCZ = cz;
    int dx = cx - centerCX;
    int dz = cz - centerCZ;

    // unsigned 范围判断：dx/dz 是否在 [-R, R]
    if ((unsigned)(dx + R) <= (unsigned)(2 * R) && (unsigned)(dz + R) <= (unsigned)(2 * R)) {
        curChunk = grid[dx + R][dz + R];
    } else {
        curChunk = nullptr;
    }
}

if (curChunk) {
    uint8_t b = curChunk->blocks[mapX & 7][mapY][mapZ & 7];
    if (b != BLOCK_AIR) {
        result.hit = true;
        result.distance = dist;
        result.blockPos = Vec3i(mapX, mapY, mapZ);
        result.block = (BlockType)b;
        result.normal = Vec3i(0, 0, 0);
        if (side == 0) result.normal.x = -stepX;
        else if (side == 1) result.normal.y = -stepY;
        else result.normal.z = -stepZ;
        return result;
        }
            }
        }
        return result;
    }

    void processGravity(uint8_t budget) {
        (void)budget;
        for (int i = 0; i < MAX_FALLING; i++) {
            if (!falling[i].active) continue;
            falling[i].vy -= GRAVITY_VAL;
            if (falling[i].vy < -0.8f) falling[i].vy = -0.8f;
            float nextY = falling[i].y + falling[i].vy;
            int wx = fastFloor(falling[i].x), wz = fastFloor(falling[i].z);
            float bottomNext = nextY - 0.5f;
            int supportY = fastFloor(bottomNext - 0.001f);
            if (supportY <= 0) { falling[i].active = false; continue; }
            BlockType support = getBlock(wx, supportY, wz);
            if (support != BLOCK_AIR && falling[i].vy < 0) {
                int placeY = supportY + 1;
                while (placeY < CHUNK_SIZE_Y && getBlock(wx, placeY, wz) != BLOCK_AIR) placeY++;
                if (placeY >= CHUNK_SIZE_Y) { falling[i].active = false; continue; }
                setBlockRaw(wx, placeY, wz, falling[i].type);
                falling[i].active = false;
                continue;
            }
            falling[i].y = nextY;
        }
    }
};

extern World g_world;
#endif
// ============== PLAYER.H ==============
#ifndef PLAYER_H
#define PLAYER_H
#include <math.h>
class Player {
public:
    Vec3f position, velocity;
    float yaw, pitch;
    bool onGround;
    BlockType selectedBlock;
    Vec3i targetPos;
    bool hasTarget;
    static constexpr float P_HEIGHT = 1.8f, E_HEIGHT = 1.6f, P_WIDTH = 0.6f;
    void save() {
        struct PlayerData { Vec3f pos; float yaw, pitch; } data = {position, yaw, pitch};
        g_storage.writeData("player.dat", (uint8_t *)&data, sizeof(data));
    }
    void load() {
        struct PlayerData { Vec3f pos; float yaw, pitch; } data;
        if (g_storage.readData("player.dat", (uint8_t *)&data, sizeof(data))) {
            position = data.pos; yaw = data.yaw; pitch = data.pitch;
        }
    }
    void init(uint32_t seed) {
        uint32_t rng = seed;
        rng = rng * 1103515245 + 12345;
        int spawnX = (rng >> 16) % 32 - 16;
        rng = rng * 1103515245 + 12345;
        int spawnZ = (rng >> 16) % 32 - 16;
        position = Vec3f(spawnX + 0.5f, 40, spawnZ + 0.5f);
        yaw = pitch = 0;
        velocity = Vec3f(0, 0, 0);
        onGround = false;
        selectedBlock = BLOCK_DIRT;
        for (int y = CHUNK_SIZE_Y - 2; y >= 1; y--) {
            if (g_world.getBlock(spawnX, y, spawnZ) != BLOCK_AIR) { position.y = y + 1 + P_HEIGHT; break; }
        }
    }
    Vec3f getEyePosition() const { return Vec3f(position.x, position.y - P_HEIGHT + E_HEIGHT, position.z); }
Vec3f getForward() const {
    float cp = g_fastMath.fastCos(pitch);
    return Vec3f(g_fastMath.fastSin(yaw) * cp, -g_fastMath.fastSin(pitch), g_fastMath.fastCos(yaw) * cp);
}
    Vec3f getRight() const { return Vec3f(cosf(yaw), 0, -sinf(yaw)); }
    bool checkCollision(float x, float y, float z) {
        float hw = P_WIDTH / 2;
        int minX = fastFloor(x - hw), maxX = fastFloor(x + hw);
        int minY = fastFloor(y - P_HEIGHT), maxY = fastFloor(y);
        int minZ = fastFloor(z - hw), maxZ = fastFloor(z + hw);
        for (int bx = minX; bx <= maxX; bx++)
            for (int by = minY; by <= maxY; by++)
                for (int bz = minZ; bz <= maxZ; bz++)
                    if (g_world.getBlockFast(bx, by, bz) != BLOCK_AIR) return true;
        return false;
    }
    void move(float dx, float dz) {
float sy = g_fastMath.fastSin(yaw);
float cy = g_fastMath.fastCos(yaw);
Vec3f fwd(sy, 0, cy), rgt(cy, 0, -sy);
        float moveX = (fwd.x * dz + rgt.x * dx) * MOVE_SPEED;
        float moveZ = (fwd.z * dz + rgt.z * dx) * MOVE_SPEED;
        float newX = position.x + moveX;
        if (!checkCollision(newX, position.y, position.z)) position.x = newX;
        float newZ = position.z + moveZ;
        if (!checkCollision(position.x, position.y, newZ)) position.z = newZ;
    }
    void turn(float dyaw, float dpitch) {
        yaw += dyaw * TURN_SPEED;
        pitch += dpitch * LOOK_SPEED;
        if (pitch > 1.4f) pitch = 1.4f;
        if (pitch < -1.4f) pitch = -1.4f;
        while (yaw > MY_PI) yaw -= 2 * MY_PI;
        while (yaw < -MY_PI) yaw += 2 * MY_PI;
    }
    void jump() { if (onGround) { velocity.y = JUMP_VELOCITY; onGround = false; } }
    void update() {
        velocity.y -= GRAVITY_VAL;
        if (velocity.y < -0.8f) velocity.y = -0.8f;
        float newY = position.y + velocity.y;
        if (checkCollision(position.x, newY, position.z)) {
            if (velocity.y < 0) {
                float feetPos = newY - P_HEIGHT;
                int blockY = fastFloor(feetPos);
                position.y = (float)blockY + 1.0f + P_HEIGHT + 0.002f;
                velocity.y = 0; onGround = true;
            } else { velocity.y = 0; }
        } else { position.y = newY; onGround = false; }
        g_world.updateAroundPlayer(position.x, position.z);
        Vec3f eye = getEyePosition(), dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        hasTarget = hit.hit;
        if (hasTarget) targetPos = hit.blockPos;
    }
    bool breakBlock() {
        RayHit hit = g_world.raycast(getEyePosition(), getForward(), 5.0f);
        if (hit.hit && hit.block != BLOCK_AIR && hit.block != BLOCK_BEDROCK) {
            g_world.setBlock(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z, BLOCK_AIR);
            return true;
        }
        return false;
    }
    bool placeBlock() {
        RayHit hit = g_world.raycast(getEyePosition(), getForward(), 5.0f);
        if (hit.hit) {
            int px = hit.blockPos.x + hit.normal.x, py = hit.blockPos.y + hit.normal.y, pz = hit.blockPos.z + hit.normal.z;
            BlockType old = g_world.getBlock(px, py, pz);
            g_world.setBlock(px, py, pz, selectedBlock);
            if (checkCollision(position.x, position.y, position.z)) {
                g_world.setBlock(px, py, pz, old);
                return false;
            }
            return true;
        }
        return false;
    }
};
extern Player g_player;
#endif
// ============== RENDER.H ==============
#ifndef RENDER_H
#define RENDER_H
class Renderer {
private:
    uint16_t frameBuffer[SCREEN_W * SCREEN_H];
    uint8_t renderStep = 4;
    // 预计算：localDirX/localDirY（未归一化），以及平方项（用于 invLen）
    float preX[SCREEN_W], preX2[SCREEN_W];
    float preY[SCREEN_H], preY2[SCREEN_H];

    float aspectRatio;
    float tanHalfFov;

    uint16_t getBlockColor(BlockType block, int side, float dist) {
        uint16_t color;
        switch (block) {
        case BLOCK_STONE: color = (side == 0) ? COLOR_STONE_TOP : COLOR_STONE_SIDE; break;
        case BLOCK_DIRT: color = (side == 0) ? COLOR_DIRT_TOP : COLOR_DIRT_SIDE; break;
        case BLOCK_GRASS: color = (side == 0) ? COLOR_GRASS_TOP : COLOR_GRASS_SIDE; break;
        case BLOCK_WOOD: color = (side == 0) ? COLOR_WOOD_TOP : COLOR_WOOD_SIDE; break;
        case BLOCK_LEAVES: color = (side == 0) ? COLOR_LEAVES_TOP : COLOR_LEAVES_SIDE; break;
        case BLOCK_BEDROCK: color = (side == 0) ? COLOR_BEDROCK_TOP : COLOR_BEDROCK_SIDE; break;
        case BLOCK_GRAVEL: color = (side == 0) ? COLOR_GRAVEL_TOP : COLOR_GRAVEL_SIDE; break;
        default: color = 0x0000; break;
        }
        if (dist > 20) {
            float fogFactor = (dist - 20) / 30.0f; if (fogFactor > 1.0f) fogFactor = 1.0f;
            uint8_t r = ((color >> 11) & 0x1F), g = ((color >> 5) & 0x3F), b = (color & 0x1F);
            uint8_t skyR = (COLOR_SKY >> 11) & 0x1F, skyG = (COLOR_SKY >> 5) & 0x3F, skyB = COLOR_SKY & 0x1F;
            r = r + (uint8_t)((skyR - r) * fogFactor); g = g + (uint8_t)((skyG - g) * fogFactor); b = b + (uint8_t)((skyB - b) * fogFactor);
            color = (r << 11) | (g << 5) | b;
        }
        return color;
    }
public:
    void init() {
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setSwapBytes(true);
    M5Cardputer.Display.fillScreen(COLOR_SKY);

    aspectRatio = (float)SCREEN_W / (float)SCREEN_H;
    tanHalfFov = tanf(FOV * 0.5f);

    // X 方向预计算
    for (int x = 0; x < SCREEN_W; x++) {
        float xNorm = (x - SCREEN_CENTER_X) / (float)SCREEN_CENTER_X;
        float lx = xNorm * tanHalfFov * aspectRatio;
        preX[x] = lx;
        preX2[x] = lx * lx;
    }
    // Y 方向预计算
    for (int y = 0; y < SCREEN_H; y++) {
        float yNorm = (SCREEN_CENTER_Y - y) / (float)SCREEN_CENTER_Y;
        float ly = yNorm * tanHalfFov;
        preY[y] = ly;
        preY2[y] = ly * ly;
    }
}
        // Step4-1: crosshair into frameBuffer
    inline void drawCrosshairToBuffer() {
        const uint16_t c = 0xFFFF;
        const int cx = SCREEN_CENTER_X;
        const int cy = SCREEN_CENTER_Y;

        // horizontal
        int x0 = cx - 5, x1 = cx + 5;
        if (x0 < 0) x0 = 0; if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
        for (int x = x0; x <= x1; x++) frameBuffer[cy * SCREEN_W + x] = c;

        // vertical
        int y0 = cy - 5, y1 = cy + 5;
        if (y0 < 0) y0 = 0; if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
        for (int y = y0; y <= y1; y++) frameBuffer[y * SCREEN_W + cx] = c;
    }
    
    void setRenderStep(uint8_t s) { if (s < 1) s = 1; if (s > 4) s = 4; renderStep = s; }
    uint8_t getRenderStep() const { return renderStep; }
    void render() {
        Vec3f eye = g_player.getEyePosition(); float yaw = g_player.yaw, pitch = g_player.pitch;
        float cosYaw = g_fastMath.fastCos(yaw),  sinYaw = g_fastMath.fastSin(yaw);
float cosPitch = g_fastMath.fastCos(pitch), sinPitch = g_fastMath.fastSin(pitch);

        float aspectRatio = (float)SCREEN_W / (float)SCREEN_H;
        float tanHalfFov = tanf(FOV * 0.5f);

        const int STEP = (int)renderStep;
        constexpr float EDGE_WIDTH = 0.05f;

        const float xScale = tanHalfFov * aspectRatio;
        const float yScale = tanHalfFov;
        const float invCenterX = 1.0f / (float)SCREEN_CENTER_X;
        const float invCenterY = 1.0f / (float)SCREEN_CENTER_Y;
        const float xStep = xScale * ((float)STEP * invCenterX);

        // 组合旋转矩阵的三列（把 yaw/pitch 旋转“增量化”的关键）
        // colX = rotate( (1,0,0) )
        const float colXx = cosYaw;
        const float colXy = 0.0f;
        const float colXz = -sinYaw;
        // colY = rotate( (0,1,0) )
        const float colYx = sinPitch * sinYaw;
        const float colYy = cosPitch;
        const float colYz = sinPitch * cosYaw;
        // colZ = rotate( (0,0,1) )
        const float colZx = cosPitch * sinYaw;
        const float colZy = -sinPitch;
        const float colZz = cosPitch * cosYaw;
        // 每列的世界空间增量（只跟 dx 和 colX 有关）
        const float dWx = xStep * colXx;
        const float dWy = xStep * colXy; // =0
        const float dWz = xStep * colXz;

        for (int screenY = 0; screenY < SCREEN_H; screenY += STEP) {

    // Step1-1：每行一次
    float yNorm = ((float)SCREEN_CENTER_Y - (float)screenY) * invCenterY;
    float yVal  = yNorm * yScale;
    float y2    = yVal * yVal;

    // baseRow = y*colY + 1*colZ
    float baseRowX = yVal * colYx + colZx;
    float baseRowY = yVal * colYy + colZy;
    float baseRowZ = yVal * colYz + colZz;

    // x 从最左开始：xNorm=-1 => xVal=-xScale
    float xVal = -xScale;

    // worldBase = x*colX + baseRow（每行一次）
    float worldBaseX = xVal * colXx + baseRowX;
    float worldBaseY = xVal * colXy + baseRowY; // colXy=0
    float worldBaseZ = xVal * colXz + baseRowZ;

    for (int screenX = 0; screenX < SCREEN_W; screenX += STEP) {

        // 归一化：长度平方仍为 x^2 + y^2 + 1（旋转不改变长度）
        float invLen = fastInvSqrt(xVal * xVal + y2 + 1.0f);
        Vec3f rayDir(worldBaseX * invLen, worldBaseY * invLen, worldBaseZ * invLen);

        RayHit hit = g_world.raycast(eye, rayDir, (float)MAX_RENDER_DIST);
        uint16_t color;
        if (hit.hit) {
            int side = (hit.normal.y > 0) ? 0 : (hit.normal.y < 0) ? 1 : (hit.normal.x != 0) ? 2 : 3;
            color = getBlockColor(hit.block, side, hit.distance);
            if (g_player.hasTarget && hit.blockPos == g_player.targetPos) {
                Vec3f hitPos = eye + rayDir * hit.distance;
                float u = 0, v = 0;
                if (side == 0 || side == 1) { u = hitPos.x - fastFloor(hitPos.x); v = hitPos.z - fastFloor(hitPos.z); }
                else if (side == 2) { u = hitPos.y - fastFloor(hitPos.y); v = hitPos.z - fastFloor(hitPos.z); }
                else { u = hitPos.x - fastFloor(hitPos.x); v = hitPos.y - fastFloor(hitPos.y); }
                if (u < EDGE_WIDTH || u > 1.0f - EDGE_WIDTH || v < EDGE_WIDTH || v > 1.0f - EDGE_WIDTH) color = 0x0000;
            }
        } else { color = COLOR_SKY; }

        int h = (screenY + STEP <= SCREEN_H) ? STEP : (SCREEN_H - screenY);
        int w = (screenX + STEP <= SCREEN_W) ? STEP : (SCREEN_W - screenX);
        for (int dy = 0; dy < h; ++dy) {
            uint16_t *row = &frameBuffer[(screenY + dy) * SCREEN_W + screenX];
            for (int dxp = 0; dxp < w; ++dxp) row[dxp] = color;
        }

        // Step1-1：每列只做增量（不再做 yaw/pitch 旋转乘法）
        worldBaseX += dWx;
        worldBaseY += dWy;
        worldBaseZ += dWz;
        xVal += xStep;
    }
}
        for (int i = 0; i < World::getMaxFallingEnts(); i++) {
            const World::FallingEnt *e = &g_world.getFallingEnts()[i];
            if (!e->active) continue;
            float rx = e->x - eye.x, ry = e->y - eye.y, rz = e->z - eye.z;
            float cx = rx * cosYaw - rz * sinYaw, cz = rx * sinYaw + rz * cosYaw;
            float cy = ry * cosPitch - cz * sinPitch, cz2 = ry * sinPitch + cz * cosPitch;
            if (cz2 <= 0.2f) continue;
            float sxNorm = (cx / cz2) / (tanHalfFov * aspectRatio);
            float syNorm = (cy / cz2) / (tanHalfFov);
            int sx = SCREEN_CENTER_X + (int)(sxNorm * SCREEN_CENTER_X);
            int sy = SCREEN_CENTER_Y - (int)(syNorm * SCREEN_CENTER_Y);
            int size = (int)((SCREEN_H / cz2) * 0.7f); if (size < 2) size = 2; if (size > 18) size = 18;
            uint16_t col = getBlockColor(e->type, 0, cz2);
            int half = size / 2, x0 = sx - half, x1 = sx + half, y0 = sy - half, y1 = sy + half;
            if (x1 < 0 || x0 >= SCREEN_W || y1 < 0 || y0 >= SCREEN_H) continue;
            if (x0 < 0) x0 = 0; if (x1 >= SCREEN_W) x1 = SCREEN_W - 1; if (y0 < 0) y0 = 0; if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
            for (int yy = y0; yy <= y1; yy++) { uint16_t *row = &frameBuffer[yy * SCREEN_W]; for (int xx = x0; xx <= x1; xx++) row[xx] = col; }
        }
        drawCrosshairToBuffer(); // Step4-1
        M5Cardputer.Display.pushImage(0, 0, SCREEN_W, SCREEN_H, frameBuffer);
    }
    void drawCrosshair() { uint16_t color = 0xFFFF; M5Cardputer.Display.drawFastHLine(SCREEN_CENTER_X - 5, SCREEN_CENTER_Y, 11, color); M5Cardputer.Display.drawFastVLine(SCREEN_CENTER_X, SCREEN_CENTER_Y - 5, 11, color); }
    void drawHUD(int fpsVal) {
    // Step4-2: update text every N frames, but draw every frame (because pushImage overwrites)
    static uint8_t tick = 0;
    static char posBuf[32];
    static char blkBuf[16];
    static char fpsBuf[16];

    if (tick == 0) {
        snprintf(posBuf, sizeof(posBuf), "%.1f %.1f %.1f", g_player.position.x, g_player.position.y, g_player.position.z);
        const char *blockNames[] = {"AIR","STONE","DIRT","GRASS","WOOD","LEAVES","BEDROCK","GRAVEL"};
        snprintf(blkBuf, sizeof(blkBuf), "[%s]", blockNames[g_player.selectedBlock]);
        snprintf(fpsBuf, sizeof(fpsBuf), "FPS:%d", fpsVal);
    }
    tick = (tick + 1) % 5;

    M5Cardputer.Display.startWrite();                 // reduce SPI fragments
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(0xFFFF, COLOR_SKY);

    // FPS top-left (replace your per-frame FPS print)
    M5Cardputer.Display.setCursor(2, 2);
    M5Cardputer.Display.print(fpsBuf);
    M5Cardputer.Display.print("   "); // erase tail if shorter

    // Block top-right
    M5Cardputer.Display.setCursor(SCREEN_W - 60, 2);
    M5Cardputer.Display.print(blkBuf);
    M5Cardputer.Display.print("   ");

    // Pos bottom-left
    M5Cardputer.Display.setCursor(2, SCREEN_H - 10);
    M5Cardputer.Display.print(posBuf);
    M5Cardputer.Display.print("   ");

    M5Cardputer.Display.endWrite();
}
};
extern Renderer g_renderer;
#endif
// ============== UI.H ==============
#ifndef UI_H
#define UI_H
enum GameState { STATE_MENU, STATE_PLAYING, STATE_PAUSED };
class UI {
private:
    char seedInput[17];
    int seedCursor, currentWorldSlot;
    bool slotHasSave;
    void drawButton(int x, int y, int w, int h, const char *text, bool selected, uint16_t color = 0x2104) {
        uint16_t borderColor = selected ? 0xFFFF : 0x8410;
        M5Cardputer.Display.fillRect(x, y, w, h, color);
        M5Cardputer.Display.drawRect(x, y, w, h, borderColor);
        int textW = strlen(text) * 6, textX = x + (w - textW) / 2, textY = y + (h - 8) / 2;
        M5Cardputer.Display.setTextColor(selected ? 0xFFFF : 0xC618);
        M5Cardputer.Display.setCursor(textX, textY);
        M5Cardputer.Display.print(text);
    }
    void randomizeSeed() {
        uint32_t r = esp_random() % 100000000;
        sprintf(seedInput, "%u", r);
        seedCursor = strlen(seedInput);
    }
public:
    GameState state;
    uint32_t parsedSeed;
    int menuSelection;
    void init() {
        state = STATE_MENU; seedCursor = 0; menuSelection = 0; currentWorldSlot = 1;
        randomizeSeed();
        checkSlot();
    }
    void checkSlot() { slotHasSave = g_storage.hasSave(currentWorldSlot); }
    void drawMenu() {
        M5Cardputer.Display.fillScreen(0x1082);
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.setTextColor(0x07E0);
        M5Cardputer.Display.setCursor(30, 5);
        M5Cardputer.Display.print("MICRO CRAFT");
        M5Cardputer.Display.drawFastHLine(20, 25, 200, 0x4208);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0xFFFF);
        M5Cardputer.Display.setCursor(20, 35);
        M5Cardputer.Display.printf("WORLD SLOT: < %d >", currentWorldSlot);
        M5Cardputer.Display.setCursor(140, 35);
        M5Cardputer.Display.setTextColor(slotHasSave ? 0x07E0 : 0xC618);
        M5Cardputer.Display.print(slotHasSave ? "[SAVED]" : "[EMPTY]");
        M5Cardputer.Display.setTextColor(0xFFFF);
        M5Cardputer.Display.setCursor(20, 55);
        M5Cardputer.Display.print("SEED:");
        M5Cardputer.Display.drawRect(60, 52, 120, 14, 0x8410);
        if (slotHasSave) {
            M5Cardputer.Display.fillRect(61, 53, 118, 12, 0x2104);
            M5Cardputer.Display.setCursor(64, 55);
            M5Cardputer.Display.setTextColor(0x8410);
            M5Cardputer.Display.print("(Locked)");
        } else {
            M5Cardputer.Display.fillRect(61, 53, 118, 12, 0x0000);
            M5Cardputer.Display.setCursor(64, 55);
            M5Cardputer.Display.setTextColor(0xFFFF);
            M5Cardputer.Display.print(seedInput);
            if ((millis() / 500) % 2 == 0) {
                int cursorX = 64 + seedCursor * 6;
                M5Cardputer.Display.drawFastVLine(cursorX, 54, 10, 0xFFFF);
            }
        }
        const char *playText = slotHasSave ? "LOAD WORLD" : "CREATE WORLD";
        drawButton(60, 80, 120, 20, playText, menuSelection == 0, slotHasSave ? 0x000F : 0x03E0);
        drawButton(60, 105, 120, 20, "CONTROLS", menuSelection == 1);
        M5Cardputer.Display.setTextColor(0x8410);
        M5Cardputer.Display.setCursor(5, 125);
        M5Cardputer.Display.print("Arrows:Slot  0-9:Seed  Enter:Go");
    }

    void drawPauseMenu() {
        for (int y = 0; y < SCREEN_H; y += 2) {
            for (int x = 0; x < SCREEN_W; x += 2) {
                M5Cardputer.Display.drawPixel(x + (y / 2) % 2, y, 0x0000);
            }
        }
        M5Cardputer.Display.fillRect(60, 40, 120, 55, 0x2104);
        M5Cardputer.Display.drawRect(60, 40, 120, 55, 0xFFFF);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0xFFFF);
        M5Cardputer.Display.setCursor(95, 48);
        M5Cardputer.Display.print("PAUSED");
        drawButton(75, 62, 90, 15, "RESUME", menuSelection == 0);
        drawButton(75, 80, 90, 15, "QUIT", menuSelection == 1);
    }

    void drawControls() {
        M5Cardputer.Display.fillScreen(0x1082);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0x07E0);
        M5Cardputer.Display.setCursor(80, 5);
        M5Cardputer.Display.print("CONTROLS");
        M5Cardputer.Display.setTextColor(0xFFFF);
        const char *controls[] = {"W/A/S/D - Move", "Q/E - Turn Left/Right", "R/F - Look Up/Down", "SPACE - Jump", "ENTER - Break Block", "/ - Place Block", "1-4 - Select Block", "ESC - Pause"};
        for (int i = 0; i < 8; i++) { M5Cardputer.Display.setCursor(30, 25 + i * 12); M5Cardputer.Display.print(controls[i]); }
        M5Cardputer.Display.setTextColor(0x8410);
        M5Cardputer.Display.setCursor(60, 123);
        M5Cardputer.Display.print("Press any key");
    }

    bool handleMenuInput() {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
            bool left = false, right = false;
            for (size_t i = 0; i < keys.word.size(); i++) {
                if (keys.word[i] == ',') left = true;
                if (keys.word[i] == '.') right = true;
            }
            if (left) { currentWorldSlot = (currentWorldSlot <= 1) ? 5 : currentWorldSlot - 1; checkSlot(); if (!slotHasSave) randomizeSeed(); }
            if (right) { currentWorldSlot = (currentWorldSlot >= 5) ? 1 : currentWorldSlot + 1; checkSlot(); if (!slotHasSave) randomizeSeed(); }
            if (!slotHasSave) {
                for (size_t i = 0; i < keys.word.size(); i++) {
                    char c = keys.word[i];
                    if (c >= '0' && c <= '9' && seedCursor < 16) { seedInput[seedCursor++] = c; seedInput[seedCursor] = '\0'; }
                }
                if (keys.del && seedCursor > 0) seedInput[--seedCursor] = '\0';
            }
            if (keys.fn) menuSelection = (menuSelection + 1) % 2;
            if (keys.enter) {
                if (menuSelection == 0) {
                    if (slotHasSave) return true;
                    parsedSeed = 0;
                    for (int i = 0; seedInput[i]; i++) parsedSeed = parsedSeed * 10 + (seedInput[i] - '0');
                    if (parsedSeed == 0) parsedSeed = 12345;
                    return true;
                } else { drawControls(); delay(3000); }
            }
        }
        return false;
    }

    int getSelectedSlot() const { return currentWorldSlot; }

    int handlePauseInput() {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
            if (keys.fn) menuSelection = (menuSelection + 1) % 2;
            if (keys.enter) return (menuSelection == 0) ? 1 : 2;
            for (size_t i = 0; i < keys.word.size(); i++) { if (keys.word[i] == '`') return 1; }
        }
        return 0;
    }
};

extern UI g_ui;

#endif
// ============== COMMAND.H ==============
#ifndef COMMAND_H
#define COMMAND_H
#include <stdlib.h>

constexpr uint32_t WARPS_MAGIC = 0x31505257;
constexpr int MAX_WARPS = 24, WARP_NAME_LEN = 12;

struct WarpPoint {
    char name[WARP_NAME_LEN];
    int32_t x, y, z;
    uint8_t used;
};

class WarpDB {
private:
    WarpPoint warps[MAX_WARPS];
public:
    void clear() { memset(warps, 0, sizeof(warps)); }
    void load() {
        clear();
        struct FileData { uint32_t magic; WarpPoint w[MAX_WARPS]; } data;
        if (!g_storage.readData("warps.dat", (uint8_t *)&data, sizeof(data))) return;
        if (data.magic != WARPS_MAGIC) return;
        memcpy(warps, data.w, sizeof(warps));
    }
    void save() {
        if (!g_storage.isAvailable()) return;
        struct FileData { uint32_t magic; WarpPoint w[MAX_WARPS]; } data;
        data.magic = WARPS_MAGIC;
        memcpy(data.w, warps, sizeof(warps));
        g_storage.writeData("warps.dat", (uint8_t *)&data, sizeof(data));
    }
    int find(const char *name) const {
        for (int i = 0; i < MAX_WARPS; i++)
            if (warps[i].used && strncmp(warps[i].name, name, WARP_NAME_LEN) == 0) return i;
        return -1;
    }
    bool addOrUpdate(const char *name, int32_t x, int32_t y, int32_t z) {
        if (!name || !name[0]) return false;
        int idx = find(name);
        if (idx < 0) {
            for (int i = 0; i < MAX_WARPS; i++) if (!warps[i].used) { idx = i; break; }
            if (idx < 0) return false;
            warps[idx].used = 1;
            strncpy(warps[idx].name, name, WARP_NAME_LEN - 1);
            warps[idx].name[WARP_NAME_LEN - 1] = '\0';
        }
        warps[idx].x = x; warps[idx].y = y; warps[idx].z = z;
        save(); return true;
    }
    const WarpPoint *get(int i) const {
        if (i < 0 || i >= MAX_WARPS) return nullptr;
        if (!warps[i].used) return nullptr;
        return &warps[i];
    }
};

extern WarpDB g_warps;

class CommandConsole {
public:
    bool active = false, showList = false;
    char buf[48];
    uint8_t len = 0;
    void open() { active = true; showList = false; len = 0; buf[0] = '\0'; }
    void close() { active = false; showList = false; }
    static bool parseInt(const char *s, int32_t *out) {
        if (!s || !*s) return false;
        char *end = nullptr;
        long v = strtol(s, &end, 10);
        if (end == s || *end != '\0') return false;
        *out = (int32_t)v; return true;
    }
    static void teleportTo(int32_t x, int32_t groundY, int32_t z) {
        g_player.position = Vec3f((float)x + 0.5f, (float)groundY + 1.0f + Player::P_HEIGHT + 0.002f, (float)z + 0.5f);
        g_player.velocity = Vec3f(0, 0, 0);
        g_player.onGround = false;
        g_world.updateAroundPlayer(g_player.position.x, g_player.position.z);
    }
    void exec() {
        char tmp[48];
        strncpy(tmp, buf, sizeof(tmp)); tmp[sizeof(tmp) - 1] = '\0';
        char *tok[4] = {0}; int n = 0;
        char *p = strtok(tmp, " ");
        while (p && n < 4) { tok[n++] = p; p = strtok(nullptr, " "); }
        if (n == 0) return;
        if (strcmp(tok[0], "tp") == 0) {
            if (n == 2) { int idx = g_warps.find(tok[1]); if (idx >= 0) { const WarpPoint *w = g_warps.get(idx); teleportTo(w->x, w->y, w->z); } }
            else if (n == 4) { int32_t x, y, z; if (parseInt(tok[1], &x) && parseInt(tok[2], &y) && parseInt(tok[3], &z)) teleportTo(x, y, z); }
        }
        else if (strcmp(tok[0], "set") == 0) {
            if (n == 2) { int32_t x = fastFloor(g_player.position.x), z = fastFloor(g_player.position.z);
                float feet = g_player.position.y - Player::P_HEIGHT - 0.002f; int32_t groundY = fastFloor(feet);
                g_warps.addOrUpdate(tok[1], x, groundY, z); }
        }
        else if (strcmp(tok[0], "list") == 0) showList = true;
    }
    void drawBar() {
        M5Cardputer.Display.fillRect(0, SCREEN_H - 18, SCREEN_W, 18, 0x0000);
        M5Cardputer.Display.drawRect(0, SCREEN_H - 18, SCREEN_W, 18, 0xFFFF);
        M5Cardputer.Display.setTextSize(1); M5Cardputer.Display.setTextColor(0xFFFF, 0x0000);
        M5Cardputer.Display.setCursor(4, SCREEN_H - 14); M5Cardputer.Display.print(">"); M5Cardputer.Display.print(buf);
    }
    void drawList() {
        M5Cardputer.Display.fillRect(8, 8, SCREEN_W - 16, SCREEN_H - 28, 0x2104);
        M5Cardputer.Display.drawRect(8, 8, SCREEN_W - 16, SCREEN_H - 28, 0xFFFF);
        M5Cardputer.Display.setTextSize(1); M5Cardputer.Display.setTextColor(0xFFFF, 0x2104);
        M5Cardputer.Display.setCursor(14, 12); M5Cardputer.Display.print("Saved Positions (tp name)");
        int y = 26;
        for (int i = 0; i < MAX_WARPS; i++) {
            const WarpPoint *w = g_warps.get(i);
            if (!w) continue;
            M5Cardputer.Display.setCursor(14, y);
            M5Cardputer.Display.printf("%s: %ld %ld %ld", w->name, (long)w->x, (long)w->y, (long)w->z);
            y += 10; if (y > SCREEN_H - 28) break;
        }
        M5Cardputer.Display.setCursor(14, SCREEN_H - 22); M5Cardputer.Display.print("Any key to close");
    }
    bool handleInput() {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
            if (showList) { close(); return true; }
            if (keys.enter) { exec(); close(); return true; }
            if (keys.del) { if (len > 0) buf[--len] = '\0'; return true; }
            for (size_t i = 0; i < keys.word.size(); i++) if (keys.word[i] == '`') { close(); return true; }
            for (size_t i = 0; i < keys.word.size(); i++) {
                char c = keys.word[i];
                if (len >= sizeof(buf) - 1) break;
                if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ' || c == '_' || c == '-')
                    { buf[len++] = c; buf[len] = '\0'; }
            }
            return true;
        }
        return false;
    }
};

extern CommandConsole g_console;

#endif
// ============== MAIN.CPP ==============
// ============== 全局对象 ==============
constexpr int8_t PerlinNoise::grad2[8][2]; constexpr int8_t PerlinNoise::grad3[12][3];
PerlinNoise g_noise; World g_world; Player g_player; Renderer g_renderer; UI g_ui; Storage g_storage; WarpDB g_warps; CommandConsole g_console;
FastMath g_fastMath;
unsigned long lastFrameTime = 0, lastFpsTime = 0; int frameCount = 0, fps = 0;

// Profiler variables
volatile uint32_t g_prof_chunkSaveMaxUs = 0;
volatile uint32_t g_prof_chunkLoadMaxUs = 0;
volatile uint32_t g_prof_chunkGenMaxUs  = 0;
static uint32_t g_prof_workSumUs = 0;
static uint32_t g_prof_workWorstUs = 0;

void handleGameInput() {
    M5Cardputer.update();
    static bool lastEnterKey = false, lastPlaceKey = false;
    Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
    float dx = 0, dz = 0; bool placeKeyPressed = false;
    for (size_t i = 0; i < keys.word.size(); i++) {
        char c = keys.word[i];
        switch (c) {
            case 'w': case 'W': dz = 1; break;
            case 's': case 'S': dz = -1; break;
            case 'a': case 'A': dx = -1; break;
            case 'd': case 'D': dx = 1; break;
            case 'q': case 'Q': g_player.turn(-1, 0); break;
            case 'e': case 'E': g_player.turn(1, 0); break;
            case 'r': case 'R': g_player.turn(0, 1); break;
            case 'f': case 'F': g_player.turn(0, -1); break;
            case ' ': g_player.jump(); break;
            case '`': g_ui.state = STATE_PAUSED; g_ui.menuSelection = 0; break;
            case '/': case '.': placeKeyPressed = true; break;
            case '1': if (keys.fn)g_renderer.setRenderStep(1); else g_player.selectedBlock = BLOCK_DIRT; break;
            case '2': if (keys.fn) g_renderer.setRenderStep(2); else g_player.selectedBlock = BLOCK_STONE; break;
            case '3': if (keys.fn) g_renderer.setRenderStep(3); else g_player.selectedBlock = BLOCK_GRASS; break;
            case '4': if (keys.fn) g_renderer.setRenderStep(4); else g_player.selectedBlock = BLOCK_WOOD; break;
            case 't': case 'T': if (keys.fn) { g_console.open(); return; } break;
            case '5': g_player.selectedBlock = BLOCK_GRAVEL; break;
            case '6': g_player.selectedBlock = BLOCK_BEDROCK; break;
        }
    }
    if (keys.enter && !lastEnterKey) g_player.breakBlock();
    lastEnterKey = keys.enter;
    if (placeKeyPressed && !lastPlaceKey) g_player.placeBlock();
    lastPlaceKey = placeKeyPressed;
    if (dx != 0 || dz != 0) g_player.move(dx, dz);
}

void setup() {
    auto cfg = M5.config(); M5Cardputer.begin(cfg);
    Serial.begin(115200); delay(200); Serial.println("Profiler Step0 Ready");
    M5Cardputer.Display.setRotation(1); M5Cardputer.Display.setBrightness(80); M5Cardputer.Display.fillScreen(0x0000);
    g_fastMath.init();g_storage.init(); g_ui.init(); g_renderer.init();
    lastFrameTime = millis(); lastFpsTime = millis();
}

void startGame(uint32_t seedInput) {
    int slot = g_ui.getSelectedSlot(); g_storage.setWorldSlot(slot); g_warps.load();
    uint32_t finalSeed = g_storage.hasSave(slot) ? g_storage.loadWorldSeed() : seedInput;
    if (!g_storage.hasSave(slot)) g_storage.saveWorldSeed(finalSeed);
    g_world.init(finalSeed); g_player.init(finalSeed); g_player.load();
    g_world.updateAroundPlayer(g_player.position.x, g_player.position.z);
    g_ui.state = STATE_PLAYING;
}

void loop() {
    uint32_t frameStartUs = micros(); // Profiler start
    unsigned long currentTime = millis();
    
    // FPS & Profiler Output Block
    frameCount++; 
    if (currentTime - lastFpsTime >= 1000) { 
        fps = frameCount; 
        
        // Print Profiler Data
        uint32_t avgUs = (frameCount > 0) ? (g_prof_workSumUs / frameCount) : 0;
        Serial.printf("FPS=%d | Avg=%.2fms Worst=%.2fms | Heap=%u | Save=%.2fms Load=%.2fms Gen=%.2fms\n", 
            fps, avgUs/1000.0f, g_prof_workWorstUs/1000.0f, ESP.getFreeHeap(),
            g_prof_chunkSaveMaxUs/1000.0f, g_prof_chunkLoadMaxUs/1000.0f, g_prof_chunkGenMaxUs/1000.0f);
            
        // Reset counters
        frameCount = 0; 
        g_prof_workSumUs = 0; 
        g_prof_workWorstUs = 0;
        g_prof_chunkSaveMaxUs = 0;
        g_prof_chunkLoadMaxUs = 0;
        g_prof_chunkGenMaxUs = 0;
        
        lastFpsTime = currentTime; 
    }

    switch (g_ui.state) {
        case STATE_MENU:
            g_ui.drawMenu();
            if (g_ui.handleMenuInput()) startGame(g_ui.parsedSeed);
            break;
        case STATE_PLAYING:
            if (g_console.active) g_console.handleInput();
            else handleGameInput();
            g_player.update(); g_world.processGravity(8);
            g_renderer.render(); g_renderer.drawHUD(fps);
            if (g_console.active) g_console.showList ? g_console.drawList() : g_console.drawBar();
            
            break;
        case STATE_PAUSED:
            g_ui.drawPauseMenu();
            int result = g_ui.handlePauseInput();
            if (result == 1) g_ui.state = STATE_PLAYING;
            else if (result == 2) {
                g_player.save(); g_world.saveAllChunks();
                g_ui.state = STATE_MENU; g_ui.init();
            }
            break;
    }

    uint32_t workUs = micros() - frameStartUs; // Profiler end
    g_prof_workSumUs += workUs;
    if (workUs > g_prof_workWorstUs) g_prof_workWorstUs = workUs;

    //unsigned long frameTime = millis() - currentTime;
    //if (frameTime < 33) delay(33 - frameTime);
    //lastFrameTime = currentTime;
}
