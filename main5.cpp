/*
  CardCraft
  A simple voxel engine for M5Cardputer
  =============== PLATFORMIO.INI ===============
  [env:m5stack-stamps3]
platform = espressif32@6.4.0
board = m5stack-stamps3
framework = arduino

lib_deps =
    m5stack/M5Cardputer@^1.0.2

build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DARDUINO_USB_CDC_ON_BOOT=1
    -O3
    -ffast-math
    -funroll-loops
    -DBOARD_HAS_PSRAM=0
    -D SCK=40
    -D MISO=39
    -D MOSI=14
    -D SS=12

board_build.partitions = huge_app.csv
monitor_speed = 115200
upload_speed = 1500000
  =============================================
*/
#include <M5Cardputer.h>
// ============== CONFIG.H ==============
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ============== 屏幕配置 ==============
constexpr int SCREEN_W = 240;
constexpr int SCREEN_H = 135;
constexpr int SCREEN_CENTER_X = SCREEN_W / 2;
constexpr int SCREEN_CENTER_Y = SCREEN_H / 2;

// ============== 世界配置 ==============
constexpr int CHUNK_SIZE_X = 8;
constexpr int CHUNK_SIZE_Y = 48; // 48格高（平衡内存和体验）
constexpr int CHUNK_SIZE_Z = 8;
constexpr int CHUNK_SIZE_XZ = CHUNK_SIZE_X;

constexpr int CHUNK_POOL_SIZE = 25; // 5x5区块池 = 75KB
constexpr int LOADED_RADIUS = 2;

constexpr int WORLD_HEIGHT = CHUNK_SIZE_Y; // 跟随 CHUNK_SIZE_Y
constexpr int SEA_LEVEL = 16;
constexpr int BASE_HEIGHT = 22;
constexpr int MIN_HEIGHT = 6;
constexpr int MAX_HEIGHT = 42;

// ============== 渲染配置 ==============
constexpr int MAX_RENDER_DIST = 32; // 减少渲染距离
constexpr int MAX_RAY_STEPS = 34;   // 增加步数以支持更远距离

// ============== 物理配置 ==============
constexpr float GRAVITY_VAL = 0.025f;
constexpr float JUMP_VELOCITY = 0.35f;
constexpr float MOVE_SPEED = 0.18f;
constexpr float TURN_SPEED = 0.08f;
constexpr float LOOK_SPEED = 0.06f;
constexpr float PLAYER_WIDTH = 0.6f;
constexpr float PLAYER_HEIGHT = 1.7f;
constexpr float EYE_HEIGHT = 1.5f;

// ============== 方块类型 ==============
enum BlockType : uint8_t
{
    BLOCK_AIR = 0,
    BLOCK_STONE = 1,
    BLOCK_DIRT = 2,
    BLOCK_GRASS = 3,
    BLOCK_WOOD = 4,
    BLOCK_LEAVES = 5,
    BLOCK_BEDROCK = 6,
    BLOCK_GRAVEL = 7,
    BLOCK_COUNT = 8
};

// ============== 颜色定义 (RGB565) ==============
constexpr uint16_t COLOR_SKY = 0x6DBF;
constexpr uint16_t COLOR_STONE_TOP = 0x8410;
constexpr uint16_t COLOR_DIRT_TOP = 0x8A22;
constexpr uint16_t COLOR_GRASS_TOP = 0x3666;
constexpr uint16_t COLOR_WOOD_TOP = 0x6180;
constexpr uint16_t COLOR_LEAVES_TOP = 0x2D83;
constexpr uint16_t COLOR_BEDROCK_TOP = 0x2104;
constexpr uint16_t COLOR_GRAVEL_TOP = 0x9492;

constexpr uint16_t COLOR_STONE_SIDE = 0x630C;
constexpr uint16_t COLOR_DIRT_SIDE = 0x6180;
constexpr uint16_t COLOR_GRASS_SIDE = 0x6180;
constexpr uint16_t COLOR_WOOD_SIDE = 0x5140;
constexpr uint16_t COLOR_LEAVES_SIDE = 0x2562;
constexpr uint16_t COLOR_BEDROCK_SIDE = 0x18C3;
constexpr uint16_t COLOR_GRAVEL_SIDE = 0x7BCF;

// ============== 杂项 ==============
constexpr int MAX_MODIFICATIONS = 256; // 减少
constexpr float MY_PI = 3.14159265f;
constexpr float FOV = 1.2f;

constexpr int SD_CS_PIN = 12;
constexpr int SD_MOSI_PIN = 14;
constexpr int SD_MISO_PIN = 39;
constexpr int SD_CLK_PIN = 40;

#endif
// ============== TYPES.H ==============
#ifndef TYPES_H
#define TYPES_H
#include <stdint.h>
#include <math.h>
// 3D向量
struct Vec3f
{
    float x, y, z;

    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vec3f operator+(const Vec3f &o) const { return Vec3f(x + o.x, y + o.y, z + o.z); }
    Vec3f operator-(const Vec3f &o) const { return Vec3f(x - o.x, y - o.y, z - o.z); }
    Vec3f operator*(float s) const { return Vec3f(x * s, y * s, z * s); }

    float length() const { return sqrtf(x * x + y * y + z * z); }
    Vec3f normalized() const
    {
        float len = length();
        if (len < 0.0001f)
            return Vec3f(0, 0, 0);
        return Vec3f(x / len, y / len, z / len);
    }
};
// 3D整数向量
struct Vec3i
{
    int x, y, z;
    Vec3i() : x(0), y(0), z(0) {}
    Vec3i(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
    bool operator==(const Vec3i &o) const { return x == o.x && y == o.y && z == o.z; }
};
// 射线命中信息
struct RayHit
{
    bool hit;
    float distance;
    Vec3i blockPos;
    Vec3i normal;
    BlockType block;

    RayHit() : hit(false), distance(0), block(BLOCK_AIR) {}
};
// 方块修改记录
struct BlockMod
{
    int16_t x, y, z;
    uint8_t blockType;
    bool valid;
};
// 快速数学函数
inline int fastFloor(float x)
{
    int xi = (int)x;
    return x < xi ? xi - 1 : xi;
}
inline float frac(float x)
{
    return x - fastFloor(x);
}
inline float clampf(float x, float minVal, float maxVal)
{
    return x < minVal ? minVal : (x > maxVal ? maxVal : x);
}
inline int clampi(int x, int minVal, int maxVal)
{
    return x < minVal ? minVal : (x > maxVal ? maxVal : x);
}
// 快速三角函数查找表
class FastMath
{
public:
    static constexpr int TABLE_SIZE = 256;
    float sinTable[TABLE_SIZE];
    float cosTable[TABLE_SIZE];

    void init()
    {
        for (int i = 0; i < TABLE_SIZE; i++)
        {
            float angle = (float)i / TABLE_SIZE * 2.0f * MY_PI;
            sinTable[i] = sinf(angle);
            cosTable[i] = cosf(angle);
        }
    }
    float fastSin(float angle) const
    {
        while (angle < 0)
            angle += 2 * MY_PI;
        while (angle >= 2 * MY_PI)
            angle -= 2 * MY_PI;
        int idx = (int)(angle / (2 * MY_PI) * TABLE_SIZE) & (TABLE_SIZE - 1);
        return sinTable[idx];
    }
    float fastCos(float angle) const
    {
        while (angle < 0)
            angle += 2 * MY_PI;
        while (angle >= 2 * MY_PI)
            angle -= 2 * MY_PI;
        int idx = (int)(angle / (2 * MY_PI) * TABLE_SIZE) & (TABLE_SIZE - 1);
        return cosTable[idx];
    }
};

#endif
// ============== NOISE.H ==============
#ifndef NOISE_H
#define NOISE_H

#include <stdint.h>

class PerlinNoise
{
private:
    uint8_t perm[512];
    uint32_t seed;

    static constexpr int8_t grad2[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};

    static constexpr int8_t grad3[12][3] = {
        {1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, 1}, {1, 0, -1}, {-1, 0, -1}, {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}};

    inline float fade(float t) const
    {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    inline float lerp(float a, float b, float t) const
    {
        return a + t * (b - a);
    }

    inline float grad2d(int hash, float x, float y) const
    {
        int h = hash & 7;
        return grad2[h][0] * x + grad2[h][1] * y;
    }

    inline float grad3d(int hash, float x, float y, float z) const
    {
        int h = hash % 12;
        return grad3[h][0] * x + grad3[h][1] * y + grad3[h][2] * z;
    }

public:
    void init(uint32_t s)
    {
        seed = s;

        for (int i = 0; i < 256; i++)
        {
            perm[i] = i;
        }

        uint32_t rng = seed;
        for (int i = 255; i > 0; i--)
        {
            rng = rng * 1103515245 + 12345;
            int j = (rng >> 16) % (i + 1);
            uint8_t tmp = perm[i];
            perm[i] = perm[j];
            perm[j] = tmp;
        }

        for (int i = 0; i < 256; i++)
        {
            perm[256 + i] = perm[i];
        }
    }

    float noise2D(float x, float y) const
    {
        int X = fastFloor(x) & 255;
        int Y = fastFloor(y) & 255;

        x -= fastFloor(x);
        y -= fastFloor(y);

        float u = fade(x);
        float v = fade(y);

        int A = perm[X] + Y;
        int B = perm[X + 1] + Y;

        float g00 = grad2d(perm[A], x, y);
        float g10 = grad2d(perm[B], x - 1, y);
        float g01 = grad2d(perm[A + 1], x, y - 1);
        float g11 = grad2d(perm[B + 1], x - 1, y - 1);

        return lerp(lerp(g00, g10, u), lerp(g01, g11, u), v);
    }

    float noise3D(float x, float y, float z) const
    {
        int X = fastFloor(x) & 255;
        int Y = fastFloor(y) & 255;
        int Z = fastFloor(z) & 255;

        x -= fastFloor(x);
        y -= fastFloor(y);
        z -= fastFloor(z);

        float u = fade(x);
        float v = fade(y);
        float w = fade(z);

        int A = perm[X] + Y;
        int AA = perm[A] + Z;
        int AB = perm[A + 1] + Z;
        int B = perm[X + 1] + Y;
        int BA = perm[B] + Z;
        int BB = perm[B + 1] + Z;

        float g000 = grad3d(perm[AA], x, y, z);
        float g100 = grad3d(perm[BA], x - 1, y, z);
        float g010 = grad3d(perm[AB], x, y - 1, z);
        float g110 = grad3d(perm[BB], x - 1, y - 1, z);
        float g001 = grad3d(perm[AA + 1], x, y, z - 1);
        float g101 = grad3d(perm[BA + 1], x - 1, y, z - 1);
        float g011 = grad3d(perm[AB + 1], x, y - 1, z - 1);
        float g111 = grad3d(perm[BB + 1], x - 1, y - 1, z - 1);

        float lerpX0 = lerp(g000, g100, u);
        float lerpX1 = lerp(g010, g110, u);
        float lerpX2 = lerp(g001, g101, u);
        float lerpX3 = lerp(g011, g111, u);

        float lerpY0 = lerp(lerpX0, lerpX1, v);
        float lerpY1 = lerp(lerpX2, lerpX3, v);

        return lerp(lerpY0, lerpY1, w);
    }

    float fractalNoise2D(float x, float y, int octaves, float persistence) const
    {
        float total = 0;
        float frequency = 1;
        float amplitude = 1;
        float maxValue = 0;

        for (int i = 0; i < octaves; i++)
        {
            total += noise2D(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2;
        }

        return total / maxValue;
    }

    float fractalNoise3D(float x, float y, float z, int octaves, float persistence) const
    {
        float total = 0;
        float frequency = 1;
        float amplitude = 1;
        float maxValue = 0;

        for (int i = 0; i < octaves; i++)
        {
            total += noise3D(x * frequency, y * frequency, z * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2;
        }

        return total / maxValue;
    }

    uint32_t getSeed() const { return seed; }
};

extern PerlinNoise g_noise;

#endif
// ============== CHUNK.H ==============
#ifndef CHUNK_H
#define CHUNK_H

#include <stdint.h>
#include <string.h>

struct Chunk
{
    int16_t cx, cz;
    uint8_t blocks[CHUNK_SIZE_X][CHUNK_SIZE_Y][CHUNK_SIZE_Z];
    bool loaded;
    bool modified;

    void clear()
    {
        // blocks 不在这里 memset：因为后续 loadChunk() 或 generate() 会完整覆盖 blocks
        loaded = false;
        modified = false;
        cx = 0;
        cz = 0;
    }

    BlockType getBlock(int lx, int ly, int lz) const
    {
        if (lx < 0 || lx >= CHUNK_SIZE_X ||
            ly < 0 || ly >= CHUNK_SIZE_Y ||
            lz < 0 || lz >= CHUNK_SIZE_Z)
        {
            return BLOCK_AIR;
        }
        return (BlockType)blocks[lx][ly][lz];
    }

    void setBlock(int lx, int ly, int lz, BlockType type)
    {
        if (lx >= 0 && lx < CHUNK_SIZE_X &&
            ly >= 0 && ly < CHUNK_SIZE_Y &&
            lz >= 0 && lz < CHUNK_SIZE_Z)
        {
            blocks[lx][ly][lz] = type;
            modified = true;
        }
    }

    // 快速哈希函数用于洞穴
    inline uint32_t hash3(int x, int y, int z, uint32_t seed) const
    {
        uint32_t h = seed;
        h ^= x * 374761393u;
        h ^= y * 668265263u;
        h ^= z * 1274126177u;
        h = (h ^ (h >> 13)) * 1103515245u;
        return h;
    }

    void generate(const PerlinNoise &noise, uint32_t worldSeed)
    {
        int worldX = cx * CHUNK_SIZE_X;
        int worldZ = cz * CHUNK_SIZE_Z;

        uint32_t rng = worldSeed ^ (cx * 73856093) ^ (cz * 19349663);

        for (int lx = 0; lx < CHUNK_SIZE_X; lx++)
        {
            for (int lz = 0; lz < CHUNK_SIZE_Z; lz++)
            {
                int wx = worldX + lx;
                int wz = worldZ + lz;

                // 基础高度
                float baseNoise = noise.fractalNoise2D(wx * 0.01f, wz * 0.01f, 3, 0.5f);

                // 山脉噪声
                float mountainNoise = noise.noise2D(wx * 0.02f + 100, wz * 0.02f + 100);
                if (mountainNoise > 0)
                {
                    mountainNoise = mountainNoise * mountainNoise * 15.0f;
                }
                else
                {
                    mountainNoise = 0;
                }

                int height = BASE_HEIGHT + (int)(baseNoise * 10.0f + mountainNoise);
                height = clampi(height, MIN_HEIGHT, MAX_HEIGHT);

                // 填充方块
                for (int ly = 0; ly < CHUNK_SIZE_Y; ly++)
                {
                    BlockType type = BLOCK_AIR;

                    if (ly == 0)
                    {
                        type = BLOCK_BEDROCK;
                    }
                    else if (ly < 3)
                    {
                        type = ((hash3(wx, ly, wz, worldSeed) & 1) == 0) ? BLOCK_BEDROCK : BLOCK_STONE;
                    }
                    else if (ly < height - 4)
                    {
                        type = BLOCK_STONE;

                        // 简化洞穴：用哈希代替3D噪声
                        if (ly > 4 && ly < height - 6)
                        {
                            uint32_t caveHash = hash3(wx >> 1, ly >> 1, wz >> 1, worldSeed + 12345);
                            if ((caveHash & 0x1F) == 0)
                            { // ~3%概率
                                type = BLOCK_AIR;
                            }
                        }

                        // 砂砾
                        if (type == BLOCK_STONE && ly < 10)
                        {
                            uint32_t gravelHash = hash3(wx, ly, wz, worldSeed + 99999);
                            if ((gravelHash & 0x7) == 0)
                            {
                                type = BLOCK_GRAVEL;
                            }
                        }
                    }
                    else if (ly < height)
                    {
                        type = BLOCK_DIRT;
                    }
                    else if (ly == height)
                    {
                        type = BLOCK_GRASS;
                    }

                    blocks[lx][ly][lz] = type;
                }

                // 树木
                rng = rng * 1103515245 + 12345;
                if (height > SEA_LEVEL && height < MAX_HEIGHT - 7)
                {
                    if ((rng & 0x7F) < 3)
                    {
                        if (lx >= 2 && lx < CHUNK_SIZE_X - 2 &&
                            lz >= 2 && lz < CHUNK_SIZE_Z - 2)
                        {
                            generateTree(lx, height + 1, lz, rng);
                        }
                    }
                }
            }
        }

        loaded = true;
        modified = false;
    }

    void generateTree(int lx, int ly, int lz, uint32_t rng)
    {
        // 1. 确定树高 (4-6格)
        int trunkHeight = 4 + ((rng >> 8) % 3);

        // 2. 生成树干
        for (int i = 0; i < trunkHeight; i++)
        {
            if (ly + i < CHUNK_SIZE_Y)
            {
                blocks[lx][ly + i][lz] = BLOCK_WOOD;
            }
        }

        // 3. 生成树叶
        // 关键修复：让树叶从树干顶端往下数第3格开始生成，保证覆盖均匀
        int leafStart = ly + trunkHeight - 3;
        int leafEnd = ly + trunkHeight;

        for (int ty = leafStart; ty <= leafEnd; ty++)
        {
            // 垂直边界检查
            if (ty < 0 || ty >= CHUNK_SIZE_Y)
                continue;

            // 计算当前层距离树顶的距离
            int distFromTop = leafEnd - ty;

            // 顶部2层半径为1 (十字形)，下面层半径为2 (宽大)
            int radius = (distFromTop <= 1) ? 1 : 2;

            for (int dx = -radius; dx <= radius; dx++)
            {
                for (int dz = -radius; dz <= radius; dz++)
                {
                    // 计算绝对坐标
                    int tx = lx + dx;
                    int tz = lz + dz;

                    // 水平边界检查 (防止数组越界)
                    if (tx < 0 || tx >= CHUNK_SIZE_X ||
                        tz < 0 || tz >= CHUNK_SIZE_Z)
                        continue;

                    // 切角逻辑：如果是宽层(半径2)，切掉四个角，使其变圆润
                    if (radius == 2 && abs(dx) == 2 && abs(dz) == 2)
                    {
                        continue;
                    }

                    // 只有是空气才放置树叶 (不要把树干切断了)
                    if (blocks[tx][ty][tz] == BLOCK_AIR)
                    {
                        blocks[tx][ty][tz] = BLOCK_LEAVES;
                    }
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

class Storage
{
private:
    bool sdAvailable;
    char worldPath[64];
    int currentSlot;

public:
    void init()
    {
        // 显式初始化 SPI 引脚 (Cardputer)
        SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
        // 使用 20MHz 提高读写速度
        sdAvailable = SD.begin(SD_CS_PIN, SPI, 20000000);

        if (sdAvailable)
        {
            if (!SD.exists("/microcraft"))
            {
                SD.mkdir("/microcraft");
            }
        }
        currentSlot = 1;
        // 默认路径，避免未选择槽位时出错
        snprintf(worldPath, sizeof(worldPath), "/microcraft/world_1");
    }

    bool isAvailable() const { return sdAvailable; }

    // 设置当前存档槽位
    void setWorldSlot(int slot)
    {
        currentSlot = slot;
        snprintf(worldPath, sizeof(worldPath), "/microcraft/world_%d", slot);

        if (sdAvailable)
        {
            if (!SD.exists(worldPath))
            {
                SD.mkdir(worldPath);
            }
        }
    }

    // 检查槽位是否有存档
    bool hasSave(int slot)
    {
        if (!sdAvailable)
            return false;
        char path[64];
        snprintf(path, sizeof(path), "/microcraft/world_%d/seed.dat", slot);
        return SD.exists(path);
    }

    // 保存/读取种子
    void saveWorldSeed(uint32_t seed)
    {
        if (!sdAvailable)
            return;
        char path[80];
        snprintf(path, sizeof(path), "%s/seed.dat", worldPath);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f)
        {
            f.write((uint8_t *)&seed, sizeof(uint32_t));
            f.close();
        }
    }

    uint32_t loadWorldSeed()
    {
        if (!sdAvailable)
            return 0;
        char path[80];
        snprintf(path, sizeof(path), "%s/seed.dat", worldPath);
        if (!SD.exists(path))
            return 0;
        File f = SD.open(path, FILE_READ);
        uint32_t seed = 0;
        if (f)
        {
            f.read((uint8_t *)&seed, sizeof(uint32_t));
            f.close();
        }
        return seed;
    }

    // 区块存取
    void saveChunk(Chunk *chunk)
    {
        if (!sdAvailable || !chunk->modified)
            return;
        char path[80];
        snprintf(path, sizeof(path), "%s/c_%d_%d.chk", worldPath, chunk->cx, chunk->cz);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f)
        {
            f.write((uint8_t *)&chunk->cx, 2);
            f.write((uint8_t *)&chunk->cz, 2);
            f.write((uint8_t *)chunk->blocks, sizeof(chunk->blocks));
            f.close();
            chunk->modified = false;
        }
    }

    bool loadChunk(Chunk *chunk)
    {
        if (!sdAvailable)
            return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/c_%d_%d.chk", worldPath, chunk->cx, chunk->cz);
        if (!SD.exists(path))
            return false;
        File f = SD.open(path, FILE_READ);
        if (!f)
            return false;
        int16_t cx, cz;
        f.read((uint8_t *)&cx, 2);
        f.read((uint8_t *)&cz, 2);
        if (cx != chunk->cx || cz != chunk->cz)
        {
            f.close();
            return false;
        }
        f.read((uint8_t *)chunk->blocks, sizeof(chunk->blocks));
        f.close();
        chunk->loaded = true;
        chunk->modified = false;
        return true;
    }

    // === 通用二进制数据读写 helper (用于 Player 调用) ===
    bool writeData(const char *filename, const uint8_t *data, size_t len)
    {
        if (!sdAvailable)
            return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        SD.remove(path);
        File f = SD.open(path, FILE_WRITE);
        if (f)
        {
            f.write(data, len);
            f.close();
            return true;
        }
        return false;
    }

    bool readData(const char *filename, uint8_t *buffer, size_t len)
    {
        if (!sdAvailable)
            return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        if (!SD.exists(path))
            return false;
        File f = SD.open(path, FILE_READ);
        if (f)
        {
            f.read(buffer, len);
            f.close();
            return true;
        }
        return false;
    }
};

extern Storage g_storage;

#endif
// ============== WORLD.H ==============
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>

class World
{
private:
    Chunk chunks[CHUNK_POOL_SIZE];
    uint32_t worldSeed;
    int centerCX, centerCZ;

    // ===== Falling blocks (gravity) =====
    static constexpr uint8_t GRAVITY_Q_SIZE = 128; // 必须是2的幂，便于取模
    struct GravPos
    {
        int16_t x, y, z;
    };
    GravPos gravQ[GRAVITY_Q_SIZE];
    uint8_t gravHead = 0, gravTail = 0;

    inline bool isFallingBlock(BlockType t) const
    {
        return t == BLOCK_GRAVEL; // 以后想加沙子就再加
    }

    inline void gravPush(int x, int y, int z)
    {
        if (y <= 0 || y >= CHUNK_SIZE_Y)
            return;
        uint8_t next = (uint8_t)((gravTail + 1) & (GRAVITY_Q_SIZE - 1));
        if (next == gravHead)
            return; // 队列满则丢弃（避免卡死/爆内存）
        gravQ[gravTail] = {(int16_t)x, (int16_t)y, (int16_t)z};
        gravTail = next;
    }

    inline bool gravPop(GravPos &out)
    {
        if (gravHead == gravTail)
            return false;
        out = gravQ[gravHead];
        gravHead = (uint8_t)((gravHead + 1) & (GRAVITY_Q_SIZE - 1));
        return true;
    }
    bool centerValid; // 新增：是否已做过首次 around-player 预加载

    int getChunkIndex(int cx, int cz) const
    {
        int wrapped_cx = ((cx % 5) + 5) % 5;
        int wrapped_cz = ((cz % 5) + 5) % 5;
        return wrapped_cz * 5 + wrapped_cx;
    }

public:
    void init(uint32_t seed)
    {
        worldSeed = seed;
        centerCX = 0;
        centerCZ = 0;
        centerValid = false;

        g_noise.init(seed);

        for (int i = 0; i < CHUNK_POOL_SIZE; i++)
        {
            chunks[i].clear();
        }
    }

    uint32_t getSeed() const { return worldSeed; }

    static int worldToChunkCoord(int w)
    {
        return w >= 0 ? w / CHUNK_SIZE_XZ : (w - CHUNK_SIZE_XZ + 1) / CHUNK_SIZE_XZ;
    }

    static int worldToLocalCoord(int w)
    {
        int local = w % CHUNK_SIZE_XZ;
        return local >= 0 ? local : local + CHUNK_SIZE_XZ;
    }

    Chunk *ensureChunk(int cx, int cz)
    {
        int idx = getChunkIndex(cx, cz);
        Chunk *chunk = &chunks[idx];

        // 如果该槽位被其他区块占用，或者还没加载
        if (!chunk->loaded || chunk->cx != cx || chunk->cz != cz)
        {

            // 1. 如果旧区块被修改过，先保存到 SD 卡
            if (chunk->loaded && chunk->modified)
            {
                g_storage.saveChunk(chunk);
            }

            // 2. 重置区块信息
            chunk->clear();
            chunk->cx = cx;
            chunk->cz = cz;

            // 3. 尝试从 SD 卡加载
            if (!g_storage.loadChunk(chunk))
            {
                // 4. 如果加载失败（是新区域），则通过算法生成
                chunk->generate(g_noise, worldSeed);
            }
        }

        return chunk;
    }

    // 退出游戏时保存所有修改过的区块
    void saveAllChunks()
    {
        for (int i = 0; i < CHUNK_POOL_SIZE; i++)
        {
            if (chunks[i].loaded && chunks[i].modified)
            {
                g_storage.saveChunk(&chunks[i]);
            }
        }
    }

    BlockType getBlock(int wx, int wy, int wz)
    {
        if (wy < 0 || wy >= CHUNK_SIZE_Y)
            return BLOCK_AIR;
        int cx = worldToChunkCoord(wx);
        int cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx);
        int lz = worldToLocalCoord(wz);
        return ensureChunk(cx, cz)->getBlock(lx, wy, lz);
    }

    void setBlock(int wx, int wy, int wz, BlockType type)
    {
        if (wy < 0 || wy >= CHUNK_SIZE_Y)
            return;

        int cx = worldToChunkCoord(wx);
        int cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx);
        int lz = worldToLocalCoord(wz);

        Chunk *chunk = ensureChunk(cx, cz);

        BlockType old = chunk->getBlock(lx, wy, lz);
        if (old == type)
            return;

        chunk->setBlock(lx, wy, lz, type);

        // === 触发重力检查 ===
        // 1) 如果挖掉/变空气：上面那格可能需要掉下来（连锁由队列+多帧budget解决）
        if (type == BLOCK_AIR)
        {
            gravPush(wx, wy + 1, wz);
        }

        // 2) 如果放的是沙砾：它自己可能需要立刻下落
        if (isFallingBlock(type))
        {
            gravPush(wx, wy, wz);
        }
    }

    void updateAroundPlayer(float px, float pz)
    {
        int newCX = worldToChunkCoord((int)px);
        int newCZ = worldToChunkCoord((int)pz);

        // 关键优化：没跨区块就不重复 ensure 25 次
        if (centerValid && newCX == centerCX && newCZ == centerCZ)
        {
            return;
        }

        for (int dx = -LOADED_RADIUS; dx <= LOADED_RADIUS; dx++)
        {
            for (int dz = -LOADED_RADIUS; dz <= LOADED_RADIUS; dz++)
            {
                ensureChunk(newCX + dx, newCZ + dz);
            }
        }
        centerCX = newCX;
        centerCZ = newCZ;
        centerValid = true;
    }
    // 热路径专用：只查已加载chunk；未加载则当作空气，绝不触发IO/生成
    BlockType getBlockFast(int wx, int wy, int wz)
    {
        if (wy < 0 || wy >= CHUNK_SIZE_Y)
            return BLOCK_AIR;

        int cx = worldToChunkCoord(wx);
        int cz = worldToChunkCoord(wz);
        int lx = worldToLocalCoord(wx);
        int lz = worldToLocalCoord(wz);

        int idx = getChunkIndex(cx, cz);
        Chunk *chunk = &chunks[idx];

        // 关键：不调用 ensureChunk
        if (!chunk->loaded || chunk->cx != cx || chunk->cz != cz)
        {
            return BLOCK_AIR;
        }
        return chunk->getBlock(lx, wy, lz);
    }
    RayHit raycast(Vec3f origin, Vec3f direction, float maxDist)
    {
        // ... (保持之前的 raycast 代码不变) ...
        // 为节省篇幅，这里请直接复制之前代码中的 raycast 实现
        RayHit result;
        result.hit = false;
        // 避免每次都归一化导致的 sqrtf 开销：
        // - 若 direction 已经是单位向量(你当前的 render() 和 getForward() 基本都是)，这里不会触发 sqrtf
        // - 只有在长度明显不为 1 时才做一次归一化，保证 dist/maxDist 的尺度一致
        float len2 = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (len2 < 1e-12f)
            return result;

        if (fabsf(len2 - 1.0f) > 1e-3f)
        {
            float invLen = 1.0f / sqrtf(len2);
            direction.x *= invLen;
            direction.y *= invLen;
            direction.z *= invLen;
        }
        int mapX = fastFloor(origin.x);
        int mapY = fastFloor(origin.y);
        int mapZ = fastFloor(origin.z);
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
        for (int i = 0; i < MAX_RAY_STEPS; i++)
        {
            if (sideDistX < sideDistY && sideDistX < sideDistZ)
            {
                sideDistX += deltaDistX;
                mapX += stepX;
                side = 0;
            }
            else if (sideDistY < sideDistZ)
            {
                sideDistY += deltaDistY;
                mapY += stepY;
                side = 1;
            }
            else
            {
                sideDistZ += deltaDistZ;
                mapZ += stepZ;
                side = 2;
            }
            if (mapY < 0 || mapY >= CHUNK_SIZE_Y)
                continue;
            float dist;
            if (side == 0)
                dist = sideDistX - deltaDistX;
            else if (side == 1)
                dist = sideDistY - deltaDistY;
            else
                dist = sideDistZ - deltaDistZ;
            if (dist > maxDist)
                break;
            BlockType block = getBlockFast(mapX, mapY, mapZ);
            if (block != BLOCK_AIR)
            {
                result.hit = true;
                result.distance = dist;
                result.blockPos = Vec3i(mapX, mapY, mapZ);
                result.block = block;
                result.normal = Vec3i(0, 0, 0);
                if (side == 0)
                    result.normal.x = -stepX;
                else if (side == 1)
                    result.normal.y = -stepY;
                else
                    result.normal.z = -stepZ;
                return result;
            }
        }
        return result;
    }
    // 每帧调用：处理有限次数的下落步进，避免尖峰
    void processGravity(uint8_t budget)
    {
        for (uint8_t i = 0; i < budget; i++)
        {
            GravPos p;
            if (!gravPop(p))
                return;

            BlockType b = getBlockFast(p.x, p.y, p.z);
            if (!isFallingBlock(b))
                continue;

            // 到底了就不落
            if (p.y <= 0)
                continue;

            BlockType below = getBlockFast(p.x, p.y - 1, p.z);
            if (below == BLOCK_AIR)
            {
                // 下落一步：上面变空气，下面变沙砾
                setBlock(p.x, p.y, p.z, BLOCK_AIR);
                setBlock(p.x, p.y - 1, p.z, b);

                // 继续下落：把新位置再放回队列
                gravPush(p.x, p.y - 1, p.z);
            }
        }
    }
};

extern World g_world;

#endif
// ============== PLAYER.H ==============
#ifndef PLAYER_H
#define PLAYER_H

#include <math.h>

class Player
{
public:
    Vec3f position;
    float yaw;
    float pitch;
    Vec3f velocity;
    bool onGround;
    BlockType selectedBlock;

    // 准星目标
    Vec3i targetPos;
    bool hasTarget;

    // 物理常量 (调整后更稳定)
    static constexpr float P_HEIGHT = 1.8f;
    static constexpr float E_HEIGHT = 1.6f;
    static constexpr float P_WIDTH = 0.6f;

    // === 存档功能 ===
    void save()
    {
        struct PlayerData
        {
            Vec3f pos;
            float yaw, pitch;
        } data;
        data.pos = position;
        data.yaw = yaw;
        data.pitch = pitch;
        g_storage.writeData("player.dat", (uint8_t *)&data, sizeof(data));
    }

    void load()
    {
        struct PlayerData
        {
            Vec3f pos;
            float yaw, pitch;
        } data;
        if (g_storage.readData("player.dat", (uint8_t *)&data, sizeof(data)))
        {
            position = data.pos;
            yaw = data.yaw;
            pitch = data.pitch;
        }
    }
    // ================

    void init(uint32_t seed)
    {
        uint32_t rng = seed;
        rng = rng * 1103515245 + 12345;
        int spawnX = (rng >> 16) % 32 - 16;
        rng = rng * 1103515245 + 12345;
        int spawnZ = (rng >> 16) % 32 - 16;

        position = Vec3f(spawnX + 0.5f, 40, spawnZ + 0.5f);
        yaw = 0;
        pitch = 0;
        velocity = Vec3f(0, 0, 0);
        onGround = false;
        selectedBlock = BLOCK_DIRT;

        // 寻找地面，避免出生在地下
        for (int y = CHUNK_SIZE_Y - 2; y >= 1; y--)
        {
            if (g_world.getBlock(spawnX, y, spawnZ) != BLOCK_AIR)
            {
                position.y = y + 1 + P_HEIGHT;
                break;
            }
        }
    }

    Vec3f getEyePosition() const
    {
        return Vec3f(position.x, position.y - P_HEIGHT + E_HEIGHT, position.z);
    }

    Vec3f getForward() const
    {
        float cp = cosf(pitch);
        return Vec3f(sinf(yaw) * cp, -sinf(pitch), cosf(yaw) * cp);
    }

    Vec3f getRight() const
    {
        return Vec3f(cosf(yaw), 0, -sinf(yaw));
    }

    bool checkCollision(float x, float y, float z)
    {
        float hw = P_WIDTH / 2;
        int minX = fastFloor(x - hw);
        int maxX = fastFloor(x + hw);
        int minY = fastFloor(y - P_HEIGHT);
        int maxY = fastFloor(y);
        int minZ = fastFloor(z - hw);
        int maxZ = fastFloor(z + hw);

        for (int bx = minX; bx <= maxX; bx++)
        {
            for (int by = minY; by <= maxY; by++)
            {
                for (int bz = minZ; bz <= maxZ; bz++)
                {
                    if (g_world.getBlockFast(bx, by, bz) != BLOCK_AIR)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void move(float dx, float dz)
    {
        Vec3f forward(sinf(yaw), 0, cosf(yaw));
        Vec3f right(cosf(yaw), 0, -sinf(yaw));

        float moveX = forward.x * dz + right.x * dx;
        float moveZ = forward.z * dz + right.z * dx;

        moveX *= MOVE_SPEED;
        moveZ *= MOVE_SPEED;

        float newX = position.x + moveX;
        // 分轴移动，防止卡墙
        if (!checkCollision(newX, position.y, position.z))
        {
            position.x = newX;
        }

        float newZ = position.z + moveZ;
        if (!checkCollision(position.x, position.y, newZ))
        {
            position.z = newZ;
        }
    }

    void turn(float dyaw, float dpitch)
    {
        yaw += dyaw * TURN_SPEED;
        pitch += dpitch * LOOK_SPEED;

        if (pitch > 1.4f)
            pitch = 1.4f;
        if (pitch < -1.4f)
            pitch = -1.4f;

        while (yaw > MY_PI)
            yaw -= 2 * MY_PI;
        while (yaw < -MY_PI)
            yaw += 2 * MY_PI;
    }

    void jump()
    {
        if (onGround)
        {
            velocity.y = JUMP_VELOCITY;
            onGround = false;
        }
    }

    void update()
    {
        // 1. 重力
        velocity.y -= GRAVITY_VAL;
        if (velocity.y < -0.8f)
            velocity.y = -0.8f;

        // 2. 预测位置
        float newY = position.y + velocity.y;

        // 3. 垂直碰撞检测
        if (checkCollision(position.x, newY, position.z))
        {
            if (velocity.y < 0)
            {
                // === 落地逻辑 (修复抖动版) ===
                // 使用嵌入地下的 newY 计算脚底所在的方块层
                float feetPos = newY - P_HEIGHT;
                int blockY = fastFloor(feetPos);

                // 将位置修正到方块顶面 + 身高 + 0.002f (Epsilon偏移)
                // 0.002f 确保我们在逻辑上"悬浮"在方块上，不会在下一帧立刻判定为"陷入"
                position.y = (float)blockY + 1.0f + P_HEIGHT + 0.002f;

                velocity.y = 0;
                onGround = true;
            }
            else
            {
                // 顶头
                velocity.y = 0;
            }
        }
        else
        {
            position.y = newY;
            onGround = false;
        }

        // 4. 更新周围世界加载
        g_world.updateAroundPlayer(position.x, position.z);

        // 5. 更新准星
        Vec3f eye = getEyePosition();
        Vec3f dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        hasTarget = hit.hit;
        if (hasTarget)
        {
            targetPos = hit.blockPos;
        }
    }

    bool breakBlock()
    {
        Vec3f eye = getEyePosition();
        Vec3f dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        if (hit.hit && hit.block != BLOCK_AIR && hit.block != BLOCK_BEDROCK)
        {
            g_world.setBlock(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z, BLOCK_AIR);
            return true;
        }
        return false;
    }

    bool placeBlock()
    {
        Vec3f eye = getEyePosition();
        Vec3f dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        if (hit.hit)
        {
            int px = hit.blockPos.x + hit.normal.x;
            int py = hit.blockPos.y + hit.normal.y;
            int pz = hit.blockPos.z + hit.normal.z;

            // 碰撞箱检查，防止把自己埋了
            BlockType old = g_world.getBlock(px, py, pz);
            g_world.setBlock(px, py, pz, selectedBlock);
            if (checkCollision(position.x, position.y, position.z))
            {
                g_world.setBlock(px, py, pz, old); // 撤销
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

#include <M5Cardputer.h>

class Renderer
{
private:
    // 全屏帧缓冲 (RGB565): 240*135*2 = 64800 bytes
    uint16_t frameBuffer[SCREEN_W * SCREEN_H];
    uint8_t renderStep = 4; // 运行时可改：1~4，默认 4
    uint16_t getBlockColor(BlockType block, int side, float dist)
    {
        uint16_t color;

        switch (block)
        {
        case BLOCK_STONE:
            color = (side == 0) ? COLOR_STONE_TOP : COLOR_STONE_SIDE;
            break;
        case BLOCK_DIRT:
            color = (side == 0) ? COLOR_DIRT_TOP : COLOR_DIRT_SIDE;
            break;
        case BLOCK_GRASS:
            color = (side == 0) ? COLOR_GRASS_TOP : COLOR_GRASS_SIDE;
            break;
        case BLOCK_WOOD:
            color = (side == 0) ? COLOR_WOOD_TOP : COLOR_WOOD_SIDE;
            break;
        case BLOCK_LEAVES:
            color = (side == 0) ? COLOR_LEAVES_TOP : COLOR_LEAVES_SIDE;
            break;
        case BLOCK_BEDROCK:
            color = (side == 0) ? COLOR_BEDROCK_TOP : COLOR_BEDROCK_SIDE;
            break;
        case BLOCK_GRAVEL:
            color = (side == 0) ? COLOR_GRAVEL_TOP : COLOR_GRAVEL_SIDE;
            break;
        default:
            color = 0x0000;
            break;
        }

        if (dist > 20)
        {
            float fogFactor = (dist - 20) / 30.0f;
            if (fogFactor > 1.0f)
                fogFactor = 1.0f;

            uint8_t r = ((color >> 11) & 0x1F);
            uint8_t g = ((color >> 5) & 0x3F);
            uint8_t b = (color & 0x1F);

            uint8_t skyR = (COLOR_SKY >> 11) & 0x1F;
            uint8_t skyG = (COLOR_SKY >> 5) & 0x3F;
            uint8_t skyB = COLOR_SKY & 0x1F;

            r = r + (uint8_t)((skyR - r) * fogFactor);
            g = g + (uint8_t)((skyG - g) * fogFactor);
            b = b + (uint8_t)((skyB - b) * fogFactor);

            color = (r << 11) | (g << 5) | b;
        }

        return color;
    }

public:
    void init()
    {
        M5Cardputer.Display.setRotation(1);
        M5Cardputer.Display.setSwapBytes(true);
        M5Cardputer.Display.fillScreen(COLOR_SKY);
    }
    void setRenderStep(uint8_t s)
    {
        if (s < 1)
            s = 1;
        if (s > 4)
            s = 4;
        renderStep = s;
    }
    uint8_t getRenderStep() const { return renderStep; }

    void render()
    {
        Vec3f eye = g_player.getEyePosition();
        float yaw = g_player.yaw;
        float pitch = g_player.pitch;

        float cosYaw = cosf(yaw);
        float sinYaw = sinf(yaw);
        float cosPitch = cosf(pitch);
        float sinPitch = sinf(pitch);

        float aspectRatio = (float)SCREEN_W / (float)SCREEN_H;
        float tanHalfFov = tanf(FOV / 2);

        // 分辨率步长
        const int STEP = (int)renderStep;
        // 线框宽度
        constexpr float EDGE_WIDTH = 0.05f;

        for (int screenY = 0; screenY < SCREEN_H; screenY += STEP)
        {
            for (int screenX = 0; screenX < SCREEN_W; screenX += STEP)
            {
                float screenYNorm = (SCREEN_CENTER_Y - screenY) / (float)SCREEN_CENTER_Y;
                float screenXNorm = (screenX - SCREEN_CENTER_X) / (float)SCREEN_CENTER_X;

                float localDirX = screenXNorm * tanHalfFov * aspectRatio;
                float localDirY = screenYNorm * tanHalfFov;
                float localDirZ = 1.0f;

                float len = sqrtf(localDirX * localDirX + localDirY * localDirY + localDirZ * localDirZ);
                localDirX /= len;
                localDirY /= len;
                localDirZ /= len;

                float tempY = localDirY * cosPitch - localDirZ * sinPitch;
                float tempZ = localDirY * sinPitch + localDirZ * cosPitch;
                localDirY = tempY;
                localDirZ = tempZ;

                float worldDirX = localDirX * cosYaw + localDirZ * sinYaw;
                float worldDirZ = -localDirX * sinYaw + localDirZ * cosYaw;

                Vec3f rayDir(worldDirX, localDirY, worldDirZ);

                RayHit hit = g_world.raycast(eye, rayDir, (float)MAX_RENDER_DIST);

                uint16_t color;
                if (hit.hit)
                {
                    int side;
                    if (hit.normal.y > 0)
                        side = 0;
                    else if (hit.normal.y < 0)
                        side = 1;
                    else if (hit.normal.x != 0)
                        side = 2;
                    else
                        side = 3;

                    color = getBlockColor(hit.block, side, hit.distance);

                    // === 仅当该方块是玩家当前指向的目标时，才绘制线框 ===
                    if (g_player.hasTarget && hit.blockPos == g_player.targetPos)
                    {
                        Vec3f hitPos = eye + rayDir * hit.distance;
                        float u = 0, v = 0;

                        // 计算UV坐标
                        if (side == 0 || side == 1)
                        {
                            u = hitPos.x - fastFloor(hitPos.x);
                            v = hitPos.z - fastFloor(hitPos.z);
                        }
                        else if (side == 2)
                        {
                            u = hitPos.y - fastFloor(hitPos.y);
                            v = hitPos.z - fastFloor(hitPos.z);
                        }
                        else
                        {
                            u = hitPos.x - fastFloor(hitPos.x);
                            v = hitPos.y - fastFloor(hitPos.y);
                        }

                        // 绘制黑色边框
                        if (u < EDGE_WIDTH || u > 1.0f - EDGE_WIDTH ||
                            v < EDGE_WIDTH || v > 1.0f - EDGE_WIDTH)
                        {
                            color = 0x0000;
                        }
                    }
                    // ===============================================
                }
                else
                {
                    color = COLOR_SKY;
                }

                // 写入帧缓冲，代替大量 fillRect() SPI 事务
                int h = (screenY + STEP <= SCREEN_H) ? STEP : (SCREEN_H - screenY);
                int w = (screenX + STEP <= SCREEN_W) ? STEP : (SCREEN_W - screenX);

                for (int dy = 0; dy < h; ++dy)
                {
                    uint16_t *row = &frameBuffer[(screenY + dy) * SCREEN_W + screenX];
                    for (int dx = 0; dx < w; ++dx)
                    {
                        row[dx] = color;
                    }
                }
            }
        }
        // 每帧只做一次大块传输
        M5Cardputer.Display.pushImage(0, 0, SCREEN_W, SCREEN_H, frameBuffer);
    }

    void drawCrosshair()
    {
        uint16_t color = 0xFFFF;
        M5Cardputer.Display.drawFastHLine(SCREEN_CENTER_X - 5, SCREEN_CENTER_Y, 11, color);
        M5Cardputer.Display.drawFastVLine(SCREEN_CENTER_X, SCREEN_CENTER_Y - 5, 11, color);
    } 

    void drawHUD()
    {
        char buf[32];

        snprintf(buf, sizeof(buf), "%.1f %.1f %.1f",
                 g_player.position.x, g_player.position.y, g_player.position.z);
        M5Cardputer.Display.setTextColor(0xFFFF, COLOR_SKY);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setCursor(2, SCREEN_H - 10);
        M5Cardputer.Display.print(buf);

        const char* blockNames[] = {"AIR","STONE","DIRT","GRASS","WOOD","LEAVES","BEDROCK","GRAVEL"};
        snprintf(buf, sizeof(buf), "[%s]", blockNames[g_player.selectedBlock]);
        M5Cardputer.Display.setCursor(SCREEN_W - 60, 2);
        M5Cardputer.Display.print(buf);
    }
};

extern Renderer g_renderer;

#endif
// ============== UI.H ==============
#ifndef UI_H
#define UI_H

#include <M5Cardputer.h>

enum GameState
{
    STATE_MENU,
    STATE_PLAYING,
    STATE_PAUSED
};

class UI
{
private:
    char seedInput[17];
    int seedCursor;
    int currentWorldSlot; // 1-5
    bool slotHasSave;

    void drawButton(int x, int y, int w, int h, const char *text, bool selected, uint16_t color = 0x2104)
    {
        uint16_t borderColor = selected ? 0xFFFF : 0x8410;
        M5Cardputer.Display.fillRect(x, y, w, h, color);
        M5Cardputer.Display.drawRect(x, y, w, h, borderColor);

        int textW = strlen(text) * 6;
        int textX = x + (w - textW) / 2;
        int textY = y + (h - 8) / 2;

        M5Cardputer.Display.setTextColor(selected ? 0xFFFF : 0xC618);
        M5Cardputer.Display.setCursor(textX, textY);
        M5Cardputer.Display.print(text);
    }

    void randomizeSeed()
    {
        // 使用 ESP32 硬件随机数生成器
        uint32_t r = esp_random() % 100000000;
        sprintf(seedInput, "%u", r);
        seedCursor = strlen(seedInput);
    }

public:
    GameState state;
    uint32_t parsedSeed;
    int menuSelection; // 0=Play, 1=Controls

    void init()
    {
        state = STATE_MENU;
        seedCursor = 0;
        menuSelection = 0;
        currentWorldSlot = 1;
        randomizeSeed(); // 默认随机种子
        checkSlot();
    }

    void checkSlot()
    {
        slotHasSave = g_storage.hasSave(currentWorldSlot);
    }

    void drawMenu()
    {
        M5Cardputer.Display.fillScreen(0x1082);

        // Title
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.setTextColor(0x07E0);
        M5Cardputer.Display.setCursor(30, 5);
        M5Cardputer.Display.print("MICRO CRAFT");
        M5Cardputer.Display.drawFastHLine(20, 25, 200, 0x4208);

        M5Cardputer.Display.setTextSize(1);

        // --- World Slot Selection ---
        M5Cardputer.Display.setTextColor(0xFFFF);
        M5Cardputer.Display.setCursor(20, 35);
        M5Cardputer.Display.printf("WORLD SLOT: < %d >", currentWorldSlot);

        // Status
        M5Cardputer.Display.setCursor(140, 35);
        if (slotHasSave)
        {
            M5Cardputer.Display.setTextColor(0x07E0);
            M5Cardputer.Display.print("[SAVED]");
        }
        else
        {
            M5Cardputer.Display.setTextColor(0xC618);
            M5Cardputer.Display.print("[EMPTY]");
        }

        // --- Seed Input ---
        M5Cardputer.Display.setTextColor(0xFFFF);
        M5Cardputer.Display.setCursor(20, 55);
        M5Cardputer.Display.print("SEED:");

        M5Cardputer.Display.drawRect(60, 52, 120, 14, 0x8410);

        if (slotHasSave)
        {
            // 如果有存档，显示"Stored"或者不可编辑的样式
            M5Cardputer.Display.fillRect(61, 53, 118, 12, 0x2104);
            M5Cardputer.Display.setCursor(64, 55);
            M5Cardputer.Display.setTextColor(0x8410);
            M5Cardputer.Display.print("(Locked)");
        }
        else
        {
            // 如果是空槽位，显示输入框
            M5Cardputer.Display.fillRect(61, 53, 118, 12, 0x0000);
            M5Cardputer.Display.setCursor(64, 55);
            M5Cardputer.Display.setTextColor(0xFFFF);
            M5Cardputer.Display.print(seedInput);

            // 光标闪烁
            if ((millis() / 500) % 2 == 0)
            {
                int cursorX = 64 + seedCursor * 6;
                M5Cardputer.Display.drawFastVLine(cursorX, 54, 10, 0xFFFF);
            }
        }

        // --- Buttons ---
        const char *playText = slotHasSave ? "LOAD WORLD" : "CREATE WORLD";
        drawButton(60, 80, 120, 20, playText, menuSelection == 0, slotHasSave ? 0x000F : 0x03E0);
        drawButton(60, 105, 120, 20, "CONTROLS", menuSelection == 1);

        // Footer
        M5Cardputer.Display.setTextColor(0x8410);
        M5Cardputer.Display.setCursor(5, 125);
        M5Cardputer.Display.print("Arrows:Slot  0-9:Seed  Enter:Go");
    }

    // ... drawPauseMenu 和 drawControls 保持不变 ...
    void drawPauseMenu()
    {
        for (int y = 0; y < SCREEN_H; y += 2)
        {
            for (int x = 0; x < SCREEN_W; x += 2)
            {
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

    void drawControls()
    {
        M5Cardputer.Display.fillScreen(0x1082);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0x07E0);
        M5Cardputer.Display.setCursor(80, 5);
        M5Cardputer.Display.print("CONTROLS");
        M5Cardputer.Display.setTextColor(0xFFFF);
        const char *controls[] = {
            "W/A/S/D - Move",
            "Q/E - Turn Left/Right",
            "R/F - Look Up/Down",
            "SPACE - Jump",
            "ENTER - Break Block",
            "/ - Place Block",
            "1-4 - Select Block",
            "ESC - Pause"};
        for (int i = 0; i < 8; i++)
        {
            M5Cardputer.Display.setCursor(30, 25 + i * 12);
            M5Cardputer.Display.print(controls[i]);
        }
        M5Cardputer.Display.setTextColor(0x8410);
        M5Cardputer.Display.setCursor(60, 123);
        M5Cardputer.Display.print("Press any key");
    }

    bool handleMenuInput()
    {
        M5Cardputer.update();

        if (M5Cardputer.Keyboard.isChange())
        {
            if (M5Cardputer.Keyboard.isPressed())
            {
                Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();

                // === 切换存档槽位 ===
                // 使用左箭头(或逗号)和右箭头(或句号)切换
                bool left = false, right = false;
                for (size_t i = 0; i < keys.word.size(); i++)
                {
                    if (keys.word[i] == ',')
                        left = true;
                    if (keys.word[i] == '.')
                        right = true;
                }
                // Cardputer的方向键映射可能比较特殊，通常是 Fn+某键
                // 这里我们假设 , 和 . 键用于切换（键盘上的 < 和 >）
                if (left)
                {
                    currentWorldSlot--;
                    if (currentWorldSlot < 1)
                        currentWorldSlot = 5;
                    checkSlot();
                    // 如果切换到了空槽，随机一个新种子
                    if (!slotHasSave)
                        randomizeSeed();
                }
                if (right)
                {
                    currentWorldSlot++;
                    if (currentWorldSlot > 5)
                        currentWorldSlot = 1;
                    checkSlot();
                    if (!slotHasSave)
                        randomizeSeed();
                }

                // === 种子输入 (仅当没存档时) ===
                if (!slotHasSave)
                {
                    for (size_t i = 0; i < keys.word.size(); i++)
                    {
                        char c = keys.word[i];
                        if (c >= '0' && c <= '9' && seedCursor < 16)
                        {
                            seedInput[seedCursor++] = c;
                            seedInput[seedCursor] = '\0';
                        }
                    }
                    if (keys.del && seedCursor > 0)
                    {
                        seedInput[--seedCursor] = '\0';
                    }
                }

                // === 菜单选择 ===
                if (keys.fn)
                {
                    menuSelection = (menuSelection + 1) % 2;
                }

                // === 确认 ===
                if (keys.enter)
                {
                    if (menuSelection == 0)
                    {
                        // 如果有存档，忽略输入框，直接返回 true (Main会去加载)
                        if (slotHasSave)
                        {
                            return true;
                        }

                        // 如果没存档，解析输入的种子
                        parsedSeed = 0;
                        for (int i = 0; seedInput[i]; i++)
                        {
                            parsedSeed = parsedSeed * 10 + (seedInput[i] - '0');
                        }
                        if (parsedSeed == 0)
                            parsedSeed = 12345;
                        return true;
                    }
                    else
                    {
                        drawControls();
                        delay(3000);
                    }
                }
            }
        }
        return false;
    }

    // 获取当前选择的槽位
    int getSelectedSlot() const { return currentWorldSlot; }

    // ... handlePauseInput 保持不变 ...
    int handlePauseInput()
    {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed())
        {
            Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
            if (keys.fn)
                menuSelection = (menuSelection + 1) % 2;
            if (keys.enter)
                return (menuSelection == 0) ? 1 : 2;
            for (size_t i = 0; i < keys.word.size(); i++)
            {
                if (keys.word[i] == '`')
                    return 1;
            }
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

constexpr uint32_t WARPS_MAGIC = 0x31505257; // 'WRP1'
constexpr int MAX_WARPS = 24;
constexpr int WARP_NAME_LEN = 12;

struct WarpPoint
{
    char name[WARP_NAME_LEN];
    int32_t x, y, z; // 记录：脚下那一层的方块坐标（地面方块Y）
    uint8_t used;
};

class WarpDB
{
private:
    WarpPoint warps[MAX_WARPS];

public:
    void clear() { memset(warps, 0, sizeof(warps)); }

    void load()
    {
        clear();
        struct FileData
        {
            uint32_t magic;
            WarpPoint w[MAX_WARPS];
        } data;

        if (!g_storage.readData("warps.dat", (uint8_t *)&data, sizeof(data)))
            return;
        if (data.magic != WARPS_MAGIC)
            return;
        memcpy(warps, data.w, sizeof(warps));
    }

    void save()
    {
        if (!g_storage.isAvailable())
            return;
        struct FileData
        {
            uint32_t magic;
            WarpPoint w[MAX_WARPS];
        } data;

        data.magic = WARPS_MAGIC;
        memcpy(data.w, warps, sizeof(warps));
        g_storage.writeData("warps.dat", (uint8_t *)&data, sizeof(data));
    }

    int find(const char *name) const
    {
        for (int i = 0; i < MAX_WARPS; i++)
        {
            if (warps[i].used && strncmp(warps[i].name, name, WARP_NAME_LEN) == 0)
                return i;
        }
        return -1;
    }

    bool addOrUpdate(const char *name, int32_t x, int32_t y, int32_t z)
    {
        if (!name || !name[0])
            return false;

        int idx = find(name);
        if (idx < 0)
        {
            for (int i = 0; i < MAX_WARPS; i++)
            {
                if (!warps[i].used)
                {
                    idx = i;
                    break;
                }
            }
            if (idx < 0)
                return false;

            warps[idx].used = 1;
            strncpy(warps[idx].name, name, WARP_NAME_LEN - 1);
            warps[idx].name[WARP_NAME_LEN - 1] = '\0';
        }

        warps[idx].x = x;
        warps[idx].y = y;
        warps[idx].z = z;
        save();
        return true;
    }

    const WarpPoint *get(int i) const
    {
        if (i < 0 || i >= MAX_WARPS)
            return nullptr;
        if (!warps[i].used)
            return nullptr;
        return &warps[i];
    }
};

extern WarpDB g_warps;

class CommandConsole
{
public:
    bool active = false;
    bool showList = false;

    char buf[48];
    uint8_t len = 0;

    void open()
    {
        active = true;
        showList = false;
        len = 0;
        buf[0] = '\0';
    }
    void close()
    {
        active = false;
        showList = false;
    }

    static bool parseInt(const char *s, int32_t *out)
    {
        if (!s || !*s)
            return false;
        char *end = nullptr;
        long v = strtol(s, &end, 10);
        if (end == s || *end != '\0')
            return false;
        *out = (int32_t)v;
        return true;
    }

    static void teleportTo(int32_t x, int32_t groundY, int32_t z)
    {
        g_player.position = Vec3f((float)x + 0.5f,
                                  (float)groundY + 1.0f + Player::P_HEIGHT + 0.002f,
                                  (float)z + 0.5f);
        g_player.velocity = Vec3f(0, 0, 0);
        g_player.onGround = false;
        g_world.updateAroundPlayer(g_player.position.x, g_player.position.z);
    }

    void exec()
    {
        // tokenize (最多4段)
        char tmp[48];
        strncpy(tmp, buf, sizeof(tmp));
        tmp[sizeof(tmp) - 1] = '\0';

        char *tok[4] = {0};
        int n = 0;
        char *p = strtok(tmp, " ");
        while (p && n < 4)
        {
            tok[n++] = p;
            p = strtok(nullptr, " ");
        }
        if (n == 0)
            return;

        if (strcmp(tok[0], "tp") == 0)
        {
            if (n == 2)
            {
                int idx = g_warps.find(tok[1]);
                if (idx >= 0)
                {
                    const WarpPoint *w = g_warps.get(idx);
                    teleportTo(w->x, w->y, w->z);
                }
            }
            else if (n == 4)
            {
                int32_t x, y, z;
                if (parseInt(tok[1], &x) && parseInt(tok[2], &y) && parseInt(tok[3], &z))
                {
                    teleportTo(x, y, z);
                }
            }
        }
        else if (strcmp(tok[0], "set") == 0)
        {
            if (n == 2)
            {
                int32_t x = fastFloor(g_player.position.x);
                int32_t z = fastFloor(g_player.position.z);
                float feet = g_player.position.y - Player::P_HEIGHT - 0.002f;
                int32_t groundY = fastFloor(feet);
                g_warps.addOrUpdate(tok[1], x, groundY, z);
            }
        }
        else if (strcmp(tok[0], "list") == 0)
        {
            showList = true;
        }
    }

    void drawBar()
    {
        M5Cardputer.Display.fillRect(0, SCREEN_H - 18, SCREEN_W, 18, 0x0000);
        M5Cardputer.Display.drawRect(0, SCREEN_H - 18, SCREEN_W, 18, 0xFFFF);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0xFFFF, 0x0000);
        M5Cardputer.Display.setCursor(4, SCREEN_H - 14);
        M5Cardputer.Display.print(">");
        M5Cardputer.Display.print(buf);
    }

    void drawList()
    {
        M5Cardputer.Display.fillRect(8, 8, SCREEN_W - 16, SCREEN_H - 28, 0x2104);
        M5Cardputer.Display.drawRect(8, 8, SCREEN_W - 16, SCREEN_H - 28, 0xFFFF);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0xFFFF, 0x2104);
        M5Cardputer.Display.setCursor(14, 12);
        M5Cardputer.Display.print("Saved Positions (tp name)");

        int y = 26;
        for (int i = 0; i < MAX_WARPS; i++)
        {
            const WarpPoint *w = g_warps.get(i);
            if (!w)
                continue;
            M5Cardputer.Display.setCursor(14, y);
            M5Cardputer.Display.printf("%s: %ld %ld %ld", w->name, (long)w->x, (long)w->y, (long)w->z);
            y += 10;
            if (y > SCREEN_H - 28)
                break;
        }
        M5Cardputer.Display.setCursor(14, SCREEN_H - 22);
        M5Cardputer.Display.print("Any key to close");
    }

    // 返回 true 表示本帧输入被 console 消费
    bool handleInput()
    {
        M5Cardputer.update();

        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed())
        {
            Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();

            if (showList)
            {
                close();
                return true;
            }

            if (keys.enter)
            {
                exec();
                close();
                return true;
            }
            if (keys.del)
            {
                if (len > 0)
                {
                    buf[--len] = '\0';
                }
                return true;
            }

            // ` 取消
            for (size_t i = 0; i < keys.word.size(); i++)
            {
                if (keys.word[i] == '`')
                {
                    close();
                    return true;
                }
            }

            // 录入字符：字母/数字/空格/下划线/横杠
            for (size_t i = 0; i < keys.word.size(); i++)
            {
                char c = keys.word[i];
                if (len >= sizeof(buf) - 1)
                    break;
                if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                    (c >= '0' && c <= '9') || c == ' ' || c == '_' || c == '-')
                {
                    buf[len++] = c;
                    buf[len] = '\0';
                }
            }
            return true;
        }
        return false;
    }
};

extern CommandConsole g_console;

#endif
// ============== MAIN.CPP ==============
#include <M5Cardputer.h>

// ============== 全局对象 ==============
constexpr int8_t PerlinNoise::grad2[8][2];
constexpr int8_t PerlinNoise::grad3[12][3];
PerlinNoise g_noise;
World g_world;
Player g_player;
Renderer g_renderer;
UI g_ui;
Storage g_storage;
WarpDB g_warps;
CommandConsole g_console;

// ============== 帧计时 ==============
unsigned long lastFrameTime = 0;
int frameCount = 0;
int fps = 0;
unsigned long lastFpsTime = 0;

// ============== 输入处理 ==============
void handleGameInput()
{
    M5Cardputer.update();

    // 防抖变量
    static bool lastEnterKey = false;
    static bool lastPlaceKey = false;

    Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();

    float dx = 0, dz = 0;
    bool placeKeyPressed = false;

    // 过滤无效输入，防止空字符触发移动
    for (size_t i = 0; i < keys.word.size(); i++)
    {
        char c = keys.word[i];

        switch (c)
        {
        case 'w':
        case 'W':
            dz = 1;
            break;
        case 's':
        case 'S':
            dz = -1;
            break;
        case 'a':
        case 'A':
            dx = -1;
            break;
        case 'd':
        case 'D':
            dx = 1;
            break;
        case 'q':
        case 'Q':
            g_player.turn(-1, 0);
            break;
        case 'e':
        case 'E':
            g_player.turn(1, 0);
            break;
        case 'r':
        case 'R':
            g_player.turn(0, 1);
            break;
        case 'f':
        case 'F':
            g_player.turn(0, -1);
            break;
        case ' ':
            g_player.jump();
            break;
        case '`': // Cardputer 上的 ESC
            g_ui.state = STATE_PAUSED;
            g_ui.menuSelection = 0;
            break;
        case '/':
        case '.':
            placeKeyPressed = true;
            break;
        case '1':
            if (keys.fn)
                g_renderer.setRenderStep(1);
            else
                g_player.selectedBlock = BLOCK_DIRT;
            break;
        case '2':
            if (keys.fn)
                g_renderer.setRenderStep(2);
            else
                g_player.selectedBlock = BLOCK_STONE;
            break;
        case '3':
            if (keys.fn)
                g_renderer.setRenderStep(3);
            else
                g_player.selectedBlock = BLOCK_GRASS;
            break;
        case '4':
            if (keys.fn)
                g_renderer.setRenderStep(4);
            else
                g_player.selectedBlock = BLOCK_WOOD;
            break;
        case 't':
        case 'T':
            if (keys.fn)
            {
                g_console.open();
                return;
            }
            break;
        case '5':
            g_player.selectedBlock = BLOCK_GRAVEL;
            break;
        case '6':
            g_player.selectedBlock = BLOCK_BEDROCK;
            break;
        }
    }

    if (keys.enter && !lastEnterKey)
    {
        g_player.breakBlock();
    }
    lastEnterKey = keys.enter;

    if (placeKeyPressed && !lastPlaceKey)
    {
        g_player.placeBlock();
    }
    lastPlaceKey = placeKeyPressed;

    if (dx != 0 || dz != 0)
    {
        g_player.move(dx, dz);
    }
}

// ============== 初始化 ==============
void setup()
{
    auto cfg = M5.config();
    M5Cardputer.begin(cfg);

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(80);
    M5Cardputer.Display.fillScreen(0x0000);

    g_storage.init();
    g_ui.init();
    g_renderer.init();

    lastFrameTime = millis();
    lastFpsTime = millis();
}

// ============== 开始新游戏 ==============
void startGame(uint32_t seedInput)
{
    // 1. 获取并设置槽位
    int slot = g_ui.getSelectedSlot();
    g_storage.setWorldSlot(slot);
    g_warps.load();
    uint32_t finalSeed = seedInput;

    // 2. 检查存档，如果有则读取旧种子
    if (g_storage.hasSave(slot))
    {
        finalSeed = g_storage.loadWorldSeed();
    }
    else
    {
        g_storage.saveWorldSeed(finalSeed);
    }

    // 3. 初始化系统
    g_world.init(finalSeed);
    g_player.init(finalSeed);

    // 4. 尝试加载玩家位置 (Player 自己调用 Storage)
    g_player.load();

    // 5. 预加载周围环境
    g_world.updateAroundPlayer(g_player.position.x, g_player.position.z);

    g_ui.state = STATE_PLAYING;
}

// ============== 主循环 ==============
void loop()
{
    unsigned long currentTime = millis();

    frameCount++;
    if (currentTime - lastFpsTime >= 1000)
    {
        fps = frameCount;
        frameCount = 0;
        lastFpsTime = currentTime;
    }

    switch (g_ui.state)
    {
    case STATE_MENU:
        g_ui.drawMenu();
        if (g_ui.handleMenuInput())
        {
            startGame(g_ui.parsedSeed);
        }
        break;

    case STATE_PLAYING:
        if (g_console.active)
        {
            g_console.handleInput(); // 吃掉键盘输入
        }
        else
        {
            handleGameInput();
        }

        g_player.update();
        g_world.processGravity(12); // 每帧最多处理12次下落步进，可按性能调 4~24
        g_renderer.render();
        g_renderer.drawCrosshair();
        g_renderer.drawHUD();

        // console overlay（在画面最后覆盖）
        if (g_console.active)
        {
            if (g_console.showList)
                g_console.drawList();
            else
                g_console.drawBar();
        }

        // FPS
        M5Cardputer.Display.setTextColor(0xFFFF, COLOR_SKY);
        M5Cardputer.Display.setCursor(2, 2);
        M5Cardputer.Display.print("FPS:");
        M5Cardputer.Display.print(fps);
        break;

    case STATE_PAUSED:
        g_ui.drawPauseMenu();
        {
            int result = g_ui.handlePauseInput();
            if (result == 1)
            {
                g_ui.state = STATE_PLAYING;
            }
            else if (result == 2)
            {
                // === 退出并保存 ===
                // Player 自己保存
                g_player.save();
                // World 保存所有区块
                g_world.saveAllChunks();

                g_ui.state = STATE_MENU;
                g_ui.init();
            }
        }
        break;
    }

    unsigned long frameTime = millis() - currentTime;
    if (frameTime < 33)
    {
        delay(33 - frameTime);
    }

    lastFrameTime = currentTime;
}
