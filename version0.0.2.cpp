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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ===== Performance feature toggles =====
// 关闭 falling blocks 的物理与渲染（提升帧率/减少每帧开销）
#ifndef ENABLE_FALLING_BLOCKS
#define ENABLE_FALLING_BLOCKS 0
#endif
// ============== CONFIG.H ==============
#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
constexpr int SCREEN_W = 240, SCREEN_H = 135, SCREEN_CENTER_X = SCREEN_W / 2, SCREEN_CENTER_Y = SCREEN_H / 2;
constexpr int CHUNK_SIZE_X = 8, CHUNK_SIZE_Y = 48, CHUNK_SIZE_Z = 8, CHUNK_SIZE_XZ = CHUNK_SIZE_X, CHUNK_POOL_SIZE = 25, LOADED_RADIUS = 2, WORLD_HEIGHT = CHUNK_SIZE_Y, SEA_LEVEL = 16, BASE_HEIGHT = 22, MIN_HEIGHT = 6, MAX_HEIGHT = 42;
constexpr int MAX_RENDER_DIST = 32, MAX_RAY_STEPS = 34;
constexpr float GRAVITY_VAL = 0.025f, JUMP_VELOCITY = 0.35f, MOVE_SPEED = 0.18f, TURN_SPEED = 0.08f, LOOK_SPEED = 0.06f, PLAYER_WIDTH = 0.6f, PLAYER_HEIGHT = 1.7f, EYE_HEIGHT = 1.5f;
// 方块类型枚举（最多 16 种，4-bit 打包）
enum BlockType : uint8_t {
    BLOCK_AIR = 0,
    BLOCK_STONE = 1,
    BLOCK_DIRT = 2,
    BLOCK_GRASS = 3,
    BLOCK_WOOD = 4,
    BLOCK_LEAVES = 5,
    BLOCK_BEDROCK = 6,
    BLOCK_GRAVEL = 7,
    BLOCK_SAND = 8,
    BLOCK_WATER = 9,
    BLOCK_COBBLESTONE = 10,
    BLOCK_PLANK = 11,
    BLOCK_BRICK = 12,
    BLOCK_GLASS = 13,
    BLOCK_IRON_ORE = 14,
    BLOCK_COAL_ORE = 15,
    BLOCK_COUNT = 16
};

// 方块颜色定义（RGB565 格式）
constexpr uint16_t COLOR_SKY = 0x6DBF;
// 原有方块 - 顶面
constexpr uint16_t COLOR_STONE_TOP = 0x8410, COLOR_DIRT_TOP = 0x8A22, COLOR_GRASS_TOP = 0x3666;
constexpr uint16_t COLOR_WOOD_TOP = 0x6180, COLOR_LEAVES_TOP = 0x2D83, COLOR_BEDROCK_TOP = 0x2104;
constexpr uint16_t COLOR_GRAVEL_TOP = 0x9492;
// 原有方块 - 侧面
constexpr uint16_t COLOR_STONE_SIDE = 0x630C, COLOR_DIRT_SIDE = 0x6180, COLOR_GRASS_SIDE = 0x6180;
constexpr uint16_t COLOR_WOOD_SIDE = 0x5140, COLOR_LEAVES_SIDE = 0x2562, COLOR_BEDROCK_SIDE = 0x18C3;
constexpr uint16_t COLOR_GRAVEL_SIDE = 0x7BCF;
// 新增方块 - 顶面
constexpr uint16_t COLOR_SAND_TOP = 0xE6D8;       // 沙黄色
constexpr uint16_t COLOR_WATER_TOP = 0x3D9F;      // 浅蓝色
constexpr uint16_t COLOR_COBBLE_TOP = 0x6B4D;     // 圆石深灰
constexpr uint16_t COLOR_PLANK_TOP = 0xBCC0;      // 木板浅棕
constexpr uint16_t COLOR_BRICK_TOP = 0xA945;      // 砖块红棕
constexpr uint16_t COLOR_GLASS_TOP = 0xBF3F;      // 玻璃浅蓝白
constexpr uint16_t COLOR_IRONORE_TOP = 0x9CD3;    // 铁矿灰粉
constexpr uint16_t COLOR_COALORE_TOP = 0x4208;    // 煤矿深灰
// 新增方块 - 侧面
constexpr uint16_t COLOR_SAND_SIDE = 0xDDD0;      // 沙黄稍暗
constexpr uint16_t COLOR_WATER_SIDE = 0x2D1F;     // 水蓝稍暗
constexpr uint16_t COLOR_COBBLE_SIDE = 0x528A;    // 圆石侧面
constexpr uint16_t COLOR_PLANK_SIDE = 0x9B80;     // 木板侧面
constexpr uint16_t COLOR_BRICK_SIDE = 0x8924;     // 砖块侧面
constexpr uint16_t COLOR_GLASS_SIDE = 0x9E5F;     // 玻璃侧面
constexpr uint16_t COLOR_IRONORE_SIDE = 0x7BCF;   // 铁矿侧面
constexpr uint16_t COLOR_COALORE_SIDE = 0x3186;   // 煤矿侧面
constexpr int MAX_MODIFICATIONS = 256, SD_CS_PIN = 12, SD_MOSI_PIN = 14, SD_MISO_PIN = 39, SD_CLK_PIN = 40;
constexpr float MY_PI = 3.14159265f, FOV = 1.2f;
constexpr float ASPECT_RATIO = (float)SCREEN_W / (float)SCREEN_H;
constexpr float TAN_HALF_FOV = 0.684136808f; // tan(FOV * 0.5f) = tan(0.6f)
constexpr float INV_CENTER_X = 1.0f / (float)SCREEN_CENTER_X;
constexpr float INV_CENTER_Y = 1.0f / (float)SCREEN_CENTER_Y;
#endif
// ============== TYPES.H ==============
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
    inline Vec3f operator+(const Vec3f &o) const { return Vec3f(x + o.x, y + o.y, z + o.z); }
    inline Vec3f operator-(const Vec3f &o) const { return Vec3f(x - o.x, y - o.y, z - o.z); }
    inline Vec3f operator*(float s) const { return Vec3f(x * s, y * s, z * s); }
    // 快速长度：用 fastInvSqrt 近似 sqrt，避免 sqrtf
    inline float length() const {
        float len2 = x * x + y * y + z * z;
        if (len2 <= 0.0f) return 0.0f;
        float invLen = fastInvSqrt(len2);
        return len2 * invLen;
    }
    inline Vec3f normalized() const { float len = length(); return len < 0.0001f ? Vec3f(0, 0, 0) : Vec3f(x / len, y / len, z / len); }
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

// 简单 16.16 定点数工具（在热点算法中尽量减少浮点）
using fix16 = int32_t;
static constexpr int FIX16_SHIFT = 16;
static constexpr fix16 FIX16_ONE = (fix16)(1 << FIX16_SHIFT);

inline fix16 floatToFix16(float v) { return (fix16)(v * (float)(1 << FIX16_SHIFT)); }
inline float fix16ToFloat(fix16 v) { return (float)v / (float)(1 << FIX16_SHIFT); }
inline fix16 fix16Mul(fix16 a, fix16 b) {
    return (fix16)(((int64_t)a * (int64_t)b) >> FIX16_SHIFT);
}
inline fix16 fix16Lerp(fix16 a, fix16 b, fix16 t) {
    // a + t * (b - a)
    return (fix16)(a + fix16Mul(t, (fix16)(b - a)));
}
inline int fix16FloorToInt(fix16 v) {
    // floor(v / 65536) with correct behavior for negatives
    int32_t i = (int32_t)(v >> FIX16_SHIFT);
    if (v < 0 && (v & (FIX16_ONE - 1))) i -= 1;
    return (int)i;
}

// ========== INVENTORY / ITEMS ==========

struct ItemStack {
    uint8_t type;   // 存 BlockType，0 表示空
    uint8_t count;  // 0 表示空
};

class Inventory {
public:
    static constexpr uint8_t HOTBAR_SLOTS = 4;
    static constexpr uint8_t BAG_SLOTS    = 12;
private:
    ItemStack hotbar[HOTBAR_SLOTS];
    ItemStack bag[BAG_SLOTS];
    uint8_t selectedHotbar = 0;
    bool open = false;

    static inline bool isEmpty(const ItemStack &s) { return s.type == (uint8_t)BLOCK_AIR || s.count == 0; }

    bool addToRange(ItemStack *slots, uint8_t n, BlockType t, uint8_t addCount) {
        // 先尝试叠加到已有同类
        for (uint8_t i = 0; i < n; ++i) {
            if (!isEmpty(slots[i]) && slots[i].type == (uint8_t)t && slots[i].count < 255) {
                uint16_t sum = (uint16_t)slots[i].count + addCount;
                if (sum > 255) sum = 255;
                slots[i].count = (uint8_t)sum;
                return true;
            }
        }
        // 找空位
        for (uint8_t i = 0; i < n; ++i) {
            if (isEmpty(slots[i])) {
                slots[i].type = (uint8_t)t;
                slots[i].count = addCount;
                return true;
            }
        }
        return false;
    }

    uint16_t countInRange(const ItemStack *slots, uint8_t n, BlockType t) const {
        uint16_t c = 0;
        for (uint8_t i = 0; i < n; ++i) {
            if (!isEmpty(slots[i]) && slots[i].type == (uint8_t)t) c += slots[i].count;
        }
        return c;
    }

    void removeFromRange(ItemStack *slots, uint8_t n, BlockType t, uint16_t removeCount) {
        for (uint8_t i = 0; i < n && removeCount > 0; ++i) {
            if (!isEmpty(slots[i]) && slots[i].type == (uint8_t)t) {
                if (slots[i].count > removeCount) {
                    slots[i].count = (uint8_t)(slots[i].count - removeCount);
                    removeCount = 0;
                } else {
                    removeCount -= slots[i].count;
                    slots[i].type = (uint8_t)BLOCK_AIR;
                    slots[i].count = 0;
                }
            }
        }
    }

public:
    void clear() {
        for (uint8_t i = 0; i < HOTBAR_SLOTS; ++i) { hotbar[i].type = (uint8_t)BLOCK_AIR; hotbar[i].count = 0; }
        for (uint8_t i = 0; i < BAG_SLOTS; ++i)    { bag[i].type    = (uint8_t)BLOCK_AIR; bag[i].count    = 0; }
        selectedHotbar = 0;
        open = false;
    }

    void initDefault() {
        clear();
        // 给玩家一些基础方块
        hotbar[0].type = (uint8_t)BLOCK_DIRT;   hotbar[0].count = 32;
        hotbar[1].type = (uint8_t)BLOCK_STONE;  hotbar[1].count = 32;
        hotbar[2].type = (uint8_t)BLOCK_GRASS;  hotbar[2].count = 16;
        hotbar[3].type = (uint8_t)BLOCK_WOOD;   hotbar[3].count = 16;
    }

    BlockType getSelectedBlock() const {
        const ItemStack &s = hotbar[selectedHotbar];
        if (isEmpty(s)) return BLOCK_AIR;
        if (s.type >= (uint8_t)BLOCK_COUNT) return BLOCK_AIR;
        return (BlockType)s.type;
    }

    void setSelectedHotbar(uint8_t idx) {
        if (idx < HOTBAR_SLOTS) selectedHotbar = idx;
    }

    uint8_t getSelectedHotbar() const { return selectedHotbar; }

    bool addItem(BlockType t, uint8_t count = 1) {
        if (t == BLOCK_AIR) return true;
        if (addToRange(hotbar, HOTBAR_SLOTS, t, count)) return true;
        return addToRange(bag, BAG_SLOTS, t, count);
    }

    bool consumeSelected(uint8_t count = 1) {
        ItemStack &s = hotbar[selectedHotbar];
        if (isEmpty(s) || s.count < count) return false;
        s.count = (uint8_t)(s.count - count);
        if (s.count == 0) { s.type = (uint8_t)BLOCK_AIR; }
        return true;
    }

    // 示例合成：每 4 个 WOOD 合成 1 个 STONE
    void craftSimple() {
        const BlockType inT  = BLOCK_WOOD;
        const BlockType outT = BLOCK_STONE;
        uint16_t have = countInRange(hotbar, HOTBAR_SLOTS, inT) + countInRange(bag, BAG_SLOTS, inT);
        if (have < 4) return;
        uint16_t crafts = have / 4;
        uint16_t needIn = crafts * 4;
        removeFromRange(hotbar, HOTBAR_SLOTS, inT, needIn);
        removeFromRange(bag, BAG_SLOTS, inT, needIn);
        for (uint16_t i = 0; i < crafts; ++i) addItem(outT, 1);
    }

    void toggleOpen() { open = !open; }
    bool isOpen() const { return open; }

    // 物品栏简易 UI：在底部显示 4 个快捷栏 + 选中框
    void drawOverlay() const {
        const int barY = SCREEN_H - 18;
        const int slotW = 52;
        const int slotH = 16;
        M5Cardputer.Display.startWrite();
        for (uint8_t i = 0; i < HOTBAR_SLOTS; ++i) {
            int x = 2 + i * (slotW + 2);
            uint16_t bg = (i == selectedHotbar) ? 0x4208 : 0x0000;
            uint16_t border = (i == selectedHotbar) ? 0xFFFF : 0x8410;
            M5Cardputer.Display.fillRect(x, barY, slotW, slotH, bg);
            M5Cardputer.Display.drawRect(x, barY, slotW, slotH, border);
            const ItemStack &s = hotbar[i];
            if (!isEmpty(s)) {
                static const char *names[] = {"AIR","STONE","DIRT","GRASS","WOOD","LEAVES","BEDROCK","GRAVEL","SAND","WATER","COBBLE","PLANK","BRICK","GLASS","IRON","COAL"};
                const char *nm = (s.type < (uint8_t)BLOCK_COUNT) ? names[s.type] : "?";
                char buf[16];
                snprintf(buf, sizeof(buf), "%s:%d", nm, s.count);
                M5Cardputer.Display.setTextColor(0xFFFF, bg);
                M5Cardputer.Display.setCursor(x + 3, barY + 4);
                M5Cardputer.Display.print(buf);
            }
        }
        M5Cardputer.Display.endWrite();
    }

    // 物品栏键盘处理：数字键选择；I 打开/关闭；C 合成
    bool handleInput(const Keyboard_Class::KeysState &keys) {
        bool changedSelection = false;
        for (size_t i = 0; i < keys.word.size(); ++i) {
            char c = keys.word[i];
            if (c >= '1' && c <= '4') {
                uint8_t idx = (uint8_t)(c - '1');
                if (idx < HOTBAR_SLOTS) {
                    setSelectedHotbar(idx);
                    changedSelection = true;
                }
            }
            if (c == 'i' || c == 'I' || c == '`') {
                // 关闭物品栏
                (void)changedSelection;
                toggleOpen();
                return true;
            }
            if (c == 'c' || c == 'C') {
                craftSimple();
                changedSelection = true;
            }
        }
        return changedSelection;
    }
};

extern Inventory g_inventory;

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
    // 使用 16.16 定点数实现 Perlin 的核心运算，减少浮点开销
    inline fix16 fadeFix(fix16 t) const {
        // t*t*t*(t*(t*6-15)+10)
        // 注意：常数乘以 FIX16_ONE 以进入定点空间
        const fix16 six   = (fix16)(6  << FIX16_SHIFT);
        const fix16 fifteen = (fix16)(15 << FIX16_SHIFT);
        const fix16 ten   = (fix16)(10 << FIX16_SHIFT);
        fix16 t2 = fix16Mul(t, t);
        fix16 t3 = fix16Mul(t2, t);
        fix16 inner = fix16Mul(t, (fix16)(fix16Mul(t, six) - fifteen)) + ten;
        return fix16Mul(t3, inner);
    }
    inline fix16 grad2dFix(int hash, fix16 x, fix16 y) const {
        int h = hash & 7;
        int gx = grad2[h][0];
        int gy = grad2[h][1];
        // gx,gy 为 -1/0/1，直接与定点数相乘
        fix16 vx = (fix16)((int32_t)gx * (int32_t)x);
        fix16 vy = (fix16)((int32_t)gy * (int32_t)y);
        return (fix16)(vx + vy);
    }
public:
    void init(uint32_t s) {
        seed=s; for(int i=0;i<256;i++) perm[i]=i;
        uint32_t rng=seed;
        for(int i=255;i>0;i--) { rng=rng*1103515245+12345; int j=(rng>>16)%(i+1); uint8_t tmp=perm[i]; perm[i]=perm[j]; perm[j]=tmp; }
        for(int i=0;i<256;i++) perm[256+i]=perm[i];
    }
    float noise2D(float x, float y) const {
        // 整数格点坐标
        int X=fastFloor(x)&255, Y=fastFloor(y)&255;
        int xfInt = fastFloor(x);
        int yfInt = fastFloor(y);
        x-=xfInt; y-=yfInt;

        // 将小数部分转换为 16.16 定点
        fix16 xf = floatToFix16(x);
        fix16 yf = floatToFix16(y);

        fix16 u=fadeFix(xf), v=fadeFix(yf);
        int A=perm[X]+Y, B=perm[X+1]+Y;
        fix16 g00=grad2dFix(perm[A],   xf,           yf);
        fix16 g10=grad2dFix(perm[B],   (fix16)(xf - FIX16_ONE), yf);
        fix16 g01=grad2dFix(perm[A+1], xf,           (fix16)(yf - FIX16_ONE));
        fix16 g11=grad2dFix(perm[B+1], (fix16)(xf - FIX16_ONE), (fix16)(yf - FIX16_ONE));

        fix16 nx0 = fix16Lerp(g00, g10, u);
        fix16 nx1 = fix16Lerp(g01, g11, u);
        fix16 nxy = fix16Lerp(nx0, nx1, v);

        // 定点结果转换回 float，保持原有接口
        return fix16ToFloat(nxy);
    }
    float fractalNoise2D(float x, float y, int octaves, float persistence) const {
        // 多重噪声叠加：幅度用定点，频率仍保持 float（次要代价）
        fix16 total = 0;
        fix16 amp = FIX16_ONE;
        fix16 maxVal = 0;
        float freq = 1.0f;
        fix16 persistenceFix = floatToFix16(persistence);
        for(int i=0;i<octaves;i++) {
            float n = noise2D(x*freq, y*freq);      // 内部已大量使用定点
            fix16 nFix = floatToFix16(n);
            total = (fix16)(total + fix16Mul(nFix, amp));
            maxVal = (fix16)(maxVal + amp);
            amp = fix16Mul(amp, persistenceFix);
            freq *= 2.0f;
        }
        if (maxVal == 0) return 0.0f;
        float totalF = fix16ToFloat(total);
        float maxF   = fix16ToFloat(maxVal);
        return totalF / maxF;
    }
};
extern PerlinNoise g_noise;
#endif
// ============== CHUNK.H ==============
#ifndef CHUNK_H
#define CHUNK_H
#include <string.h>
struct Chunk {
    static_assert(BLOCK_COUNT <= 16, "4-bit packing requires BLOCK_COUNT <= 16");
    int16_t cx, cz;
    // Step5-1/2: 1D + 4-bit packed blocks (3072 voxels -> 1536 bytes)
    static constexpr int VOXELS = CHUNK_SIZE_X * CHUNK_SIZE_Y * CHUNK_SIZE_Z; // 3072
    static constexpr int PACKED_BYTES = (VOXELS + 1) / 2;                    // 1536
    uint8_t blocks[PACKED_BYTES];

    bool loaded, modified;

    void clear() { loaded = false; modified = false; cx = 0; cz = 0; }

    static inline uint16_t vidx(int lx, int ly, int lz) {
        // x fastest: idx = ((ly*Z + lz)*X + lx)
        return (uint16_t)(((ly * CHUNK_SIZE_Z) + lz) * CHUNK_SIZE_X + lx);
    }

    // 超快：无边界检查/不改 modified（给生成、raycast 用）
    inline uint8_t getBlockFast(int lx, int ly, int lz) const {
        uint16_t i = vidx(lx, ly, lz);
        uint8_t p = blocks[i >> 1];
        return (i & 1) ? (p >> 4) : (p & 0x0F);
    }
    inline void setBlockFast(int lx, int ly, int lz, uint8_t v) {
        uint16_t i = vidx(lx, ly, lz);
        uint8_t &p = blocks[i >> 1];
        if (i & 1) p = (uint8_t)((p & 0x0F) | ((v & 0x0F) << 4));
        else       p = (uint8_t)((p & 0xF0) | (v & 0x0F));
    }

    // 原接口保留：带边界检查 + 标记 modified
    BlockType getBlock(int lx, int ly, int lz) const {
        if (lx < 0 || lx >= CHUNK_SIZE_X || ly < 0 || ly >= CHUNK_SIZE_Y || lz < 0 || lz >= CHUNK_SIZE_Z) return BLOCK_AIR;
        return (BlockType)getBlockFast(lx, ly, lz);
    }
    void setBlock(int lx, int ly, int lz, BlockType type) {
        if (lx >= 0 && lx < CHUNK_SIZE_X && ly >= 0 && ly < CHUNK_SIZE_Y && lz >= 0 && lz < CHUNK_SIZE_Z) {
            setBlockFast(lx, ly, lz, (uint8_t)type);
            modified = true;
        }
    }
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
                        // 洞穴
                        if (ly > 4 && ly < height - 6 && ((hash3(wx >> 1, ly >> 1, wz >> 1, worldSeed + 12345) & 0x1F) == 0)) type = BLOCK_AIR;
                        // 砂砾
                        if (type == BLOCK_STONE && ly < 10 && ((hash3(wx, ly, wz, worldSeed + 99999) & 0x7) == 0)) type = BLOCK_GRAVEL;
                        // 煤矿（较常见，y<25）
                        if (type == BLOCK_STONE && ly < 25 && ((hash3(wx, ly, wz, worldSeed + 11111) & 0xF) == 0)) type = BLOCK_COAL_ORE;
                        // 铁矿（较少，y<20）
                        if (type == BLOCK_STONE && ly < 20 && ((hash3(wx, ly, wz, worldSeed + 22222) & 0x1F) == 0)) type = BLOCK_IRON_ORE;
                    }
                    else if (ly < height) type = BLOCK_DIRT;
                    else if (ly == height) {
                        // 低海拔区域生成沙子（模拟海滩）
                        if (height <= SEA_LEVEL + 2) type = BLOCK_SAND;
                        else type = BLOCK_GRASS;
                    }
                    setBlockFast(lx, ly, lz, (uint8_t)type);
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
        for (int i = 0; i < trunkHeight && ly + i < CHUNK_SIZE_Y; i++) setBlockFast(lx, ly + i, lz, (uint8_t)BLOCK_WOOD);
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
                    if (getBlockFast(tx, ty, tz) == BLOCK_AIR) setBlockFast(tx, ty, tz, (uint8_t)BLOCK_LEAVES);
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
    bool removeDirRecursive(const char *path) {
        File dir = SD.open(path);
        if (!dir) return false;

        File f = dir.openNextFile();
        while (f) {
            char child[96];
            snprintf(child, sizeof(child), "%s/%s", path, f.name());
            if (f.isDirectory()) {
                f.close();
                removeDirRecursive(child);
                SD.rmdir(child);
            } else {
                f.close();
                SD.remove(child);
            }
            f = dir.openNextFile();
        }
        dir.close();
        return true;
    }
    // ===== Step6-1: async save queue (slice write) =====
    static constexpr uint8_t SAVE_Q = 4;
    static constexpr uint16_t SAVE_SLICE = 256;

    struct SaveJob {
        int16_t cx, cz;
        uint8_t data[Chunk::PACKED_BYTES];
        bool used;
    };

    SaveJob saveQ[SAVE_Q];
    uint8_t qHead = 0, qTail = 0, qCount = 0;

    bool saveActive = false;
    File saveFile;
    int16_t actCX = 0, actCZ = 0;
    uint16_t actOff = 0;
    bool headerDone = false;

    bool openActiveSaveFile() {
        char path[96];
        snprintf(path, sizeof(path), "%s/c_%d_%d.chk", worldPath, actCX, actCZ);
        saveFile = SD.open(path, FILE_WRITE); // FILE_WRITE 会截断，没必要 SD.remove
        return (bool)saveFile;
    }
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
    bool deleteWorldSlot(int slot) {
        if (!sdAvailable) return false;
        char path[64];
        snprintf(path, sizeof(path), "/microcraft/world_%d", slot);
        if (!SD.exists(path)) return true;
        removeDirRecursive(path);
        return SD.rmdir(path);
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
        size_t remain = f.size() - f.position();
        const size_t oldBytes = CHUNK_SIZE_X * CHUNK_SIZE_Y * CHUNK_SIZE_Z; // 3072
        const size_t newBytes = sizeof(chunk->blocks);                      // 1536

        if (remain == newBytes) {
            f.read((uint8_t*)chunk->blocks, newBytes);
        } else if (remain == oldBytes) {
            // 兼容旧存档：读取 3072 字节，然后打包成 4-bit
            static uint8_t tmpOld[CHUNK_SIZE_X * CHUNK_SIZE_Y * CHUNK_SIZE_Z];
            f.read(tmpOld, oldBytes);
            memset(chunk->blocks, 0, newBytes);
            for (int i = 0; i < (int)oldBytes; i++) {
                uint8_t v = tmpOld[i] & 0x0F;
                uint8_t &p = chunk->blocks[i >> 1];
                if (i & 1) p = (uint8_t)((p & 0x0F) | (v << 4));
                else       p = (uint8_t)((p & 0xF0) | v);
            }
        } else {
            f.close();
            return false;
        }
        f.close();
        chunk->loaded = true;
        chunk->modified = false;
        return true;
    }
    bool writeData(const char *filename, const uint8_t *data, size_t len) {
        if (!sdAvailable) return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        File f = SD.open(path, FILE_WRITE);
        if (f) {
            f.write(data, len);
            f.close();
            return true;
        }
        return false;
    }
    bool readData(const char *filename, uint8_t *buffer, size_t len) {
        if (!sdAvailable) return false;
        char path[80];
        snprintf(path, sizeof(path), "%s/%s", worldPath, filename);
        if (!SD.exists(path)) return false;
        File f = SD.open(path, FILE_READ);
        if (f) {
            f.read(buffer, len);
            f.close();
            return true;
        }
        return false;
    }
    // 入队：只做 memcpy，不做 SD 写
    bool queueChunkSave(const Chunk *chunk) {
        if (!sdAvailable || !chunk || !chunk->loaded || !chunk->modified) return true;

        // 队列满：为保证不丢数据，退化为同步保存（极少发生）
        if (qCount >= SAVE_Q) {
            saveChunk((Chunk*)chunk);
            return false;
        }

        SaveJob &j = saveQ[qHead];
        j.cx = chunk->cx;
        j.cz = chunk->cz;
        memcpy(j.data, chunk->blocks, Chunk::PACKED_BYTES);
        j.used = true;

        qHead = (uint8_t)((qHead + 1) % SAVE_Q);
        qCount++;
        return true;
    }

    // 每帧调用：写最多 slices 次，每次写 256B
    void processSaveQueue(uint8_t slices = 1) {
        if (!sdAvailable) return;

        while (slices--) {
            if (!saveActive) {
                if (qCount == 0) return;
                SaveJob &j = saveQ[qTail];
                if (!j.used) { qTail = (uint8_t)((qTail + 1) % SAVE_Q); qCount--; continue; }

                actCX = j.cx; actCZ = j.cz;
                actOff = 0;
                headerDone = false;
                saveActive = true;
                if (!openActiveSaveFile()) {
                    // 打不开就丢弃该任务（避免死循环）
                    j.used = false;
                    qTail = (uint8_t)((qTail + 1) % SAVE_Q);
                    qCount--;
                    saveActive = false;
                    return;
                }
            }

            SaveJob &j = saveQ[qTail];

            if (!headerDone) {
                saveFile.write((uint8_t*)&j.cx, 2);
                saveFile.write((uint8_t*)&j.cz, 2);
                headerDone = true;
            }

            uint16_t remain = (uint16_t)(Chunk::PACKED_BYTES - actOff);
            uint16_t n = (remain > SAVE_SLICE) ? SAVE_SLICE : remain;
            saveFile.write(j.data + actOff, n);
            actOff = (uint16_t)(actOff + n);

            if (actOff >= Chunk::PACKED_BYTES) {
                saveFile.close();
                j.used = false;
                qTail = (uint8_t)((qTail + 1) % SAVE_Q);
                qCount--;
                saveActive = false;
            }
        }
    }

    void flushSaveQueue() {
        // 写到空：一次性把队列全部写完（退出/存档时用）
        while (qCount > 0 || saveActive) processSaveQueue(8);
    }
};
extern Storage g_storage;
#endif
// ============== WORLD.H ==============
#ifndef WORLD_H
#define WORLD_H
#include <Arduino.h>

extern volatile uint32_t g_prof_chunkSaveMaxUs, g_prof_chunkLoadMaxUs, g_prof_chunkGenMaxUs;
enum ChunkTaskType { CHUNK_TASK_LOAD, CHUNK_TASK_GENERATE, CHUNK_TASK_SAVE_QUEUE };
struct ChunkTask { ChunkTaskType type; Chunk *chunk; int16_t cx, cz; uint32_t worldSeed; };

class World {
public:
    static constexpr uint8_t MAX_FALLING = 12;
    // y/vy 用 16.16 定点，减少每帧重力更新的浮点运算
    struct FallingEnt { float x, z; fix16 y, vy; BlockType type; bool active; };
    const FallingEnt *getFallingEnts() const { return falling; }
    static constexpr uint8_t getMaxFallingEnts() { return MAX_FALLING; }
    
    // 简单生物系统
    static constexpr uint8_t MAX_MOBS = 8;
    struct Mob {
        float x, y, z;
        float vx, vz; // 水平速度
        uint8_t type; // 0=空, 1=简单生物
        bool active;
        uint32_t lastMoveTime;
    };
    const Mob *getMobs() const { return mobs; }
    static constexpr uint8_t getMaxMobs() { return MAX_MOBS; }
private:
    Chunk chunks[CHUNK_POOL_SIZE];
    uint32_t worldSeed;
    int centerCX, centerCZ;
    bool centerValid;
    // 新增：已加载区块指针网格（固定 5x5，对应 LOADED_RADIUS=2）
    Chunk *grid[5][5];

    FallingEnt falling[MAX_FALLING];
    Mob mobs[MAX_MOBS];
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
#if !ENABLE_FALLING_BLOCKS
        (void)wx; (void)wy; (void)wz; (void)type;
        return false;
#else
        for (int i = 0; i < MAX_FALLING; i++) {
            if (!falling[i].active) {
                setBlockRaw(wx, wy, wz, BLOCK_AIR);
                falling[i].x = (float)wx + 0.5f;
                falling[i].z = (float)wz + 0.5f;
                falling[i].y = floatToFix16((float)wy + 0.5f);
                falling[i].vy = 0;
                falling[i].type = type;
                falling[i].active = true;
                return true;
            }
        }
        return false;
#endif
    }
public:
    SemaphoreHandle_t chunkMutex = nullptr;
    QueueHandle_t chunkTaskQueue = nullptr;
    void init(uint32_t seed) {
        worldSeed = seed; centerCX = 0; centerCZ = 0; centerValid = false;
        g_noise.init(seed);
        for (int i = 0; i < CHUNK_POOL_SIZE; i++) chunks[i].clear();
        for (int i = 0; i < MAX_FALLING; i++) falling[i].active = false;
        for (int i = 0; i < MAX_MOBS; i++) {
            mobs[i].active = false;
            mobs[i].type = 0;
            mobs[i].lastMoveTime = 0;
        }
        for (int gz = 0; gz < 5; gz++)
            for (int gx = 0; gx < 5; gx++)
                grid[gx][gz] = nullptr;
        // 初始化互斥锁和队列（如果还未初始化）
        if (chunkMutex == nullptr) {
            chunkMutex = xSemaphoreCreateMutex();
        }
        if (chunkTaskQueue == nullptr) chunkTaskQueue = xQueueCreate(16, sizeof(ChunkTask));
    }
    
    // 在玩家附近生成生物
    void spawnMobsNearPlayer(float px, float py, float pz) {
        uint32_t now = millis();
        int activeCount = 0;
        for (int i = 0; i < MAX_MOBS; i++) {
            if (mobs[i].active) activeCount++;
        }
        // 最多 4 个生物
        if (activeCount >= 4) return;
        
        // 每 5 秒尝试生成一个
        static uint32_t lastSpawnTime = 0;
        if (now - lastSpawnTime < 5000) return;
        lastSpawnTime = now;
        
        // 在玩家周围随机位置生成
        uint32_t rng = worldSeed ^ (uint32_t)px ^ (uint32_t)pz ^ now;
        for (int attempt = 0; attempt < 3; attempt++) {
            rng = rng * 1103515245 + 12345;
            float offsetX = ((float)(rng & 0xFF) / 255.0f - 0.5f) * 16.0f;
            rng = rng * 1103515245 + 12345;
            float offsetZ = ((float)(rng & 0xFF) / 255.0f - 0.5f) * 16.0f;
            int spawnX = (int)(px + offsetX);
            int spawnZ = (int)(pz + offsetZ);
            
            // 找到地面高度
            int spawnY = CHUNK_SIZE_Y - 1;
            for (int y = CHUNK_SIZE_Y - 1; y >= 0; y--) {
                if (getBlockFast(spawnX, y, spawnZ) != BLOCK_AIR) {
                    spawnY = y + 1;
                    break;
                }
            }
            if (spawnY >= CHUNK_SIZE_Y || spawnY < 1) continue;
            
            // 检查生成位置是否安全（上方有空间）
            if (getBlockFast(spawnX, spawnY, spawnZ) != BLOCK_AIR ||
                getBlockFast(spawnX, spawnY + 1, spawnZ) != BLOCK_AIR) continue;
            
            // 找到空闲槽位
            for (int i = 0; i < MAX_MOBS; i++) {
                if (!mobs[i].active) {
                    mobs[i].x = (float)spawnX + 0.5f;
                    mobs[i].y = (float)spawnY + 0.5f;
                    mobs[i].z = (float)spawnZ + 0.5f;
                    mobs[i].vx = 0;
                    mobs[i].vz = 0;
                    mobs[i].type = 1;
                    mobs[i].active = true;
                    mobs[i].lastMoveTime = now;
                    return;
                }
            }
        }
    }
    
    // 更新生物 AI 和移动
    void updateMobs(float playerX, float playerY, float playerZ) {
        uint32_t now = millis();
        for (int i = 0; i < MAX_MOBS; i++) {
            if (!mobs[i].active) continue;
            
            Mob &m = mobs[i];
            
            // 简单 AI：随机游走，偶尔向玩家移动
            if (now - m.lastMoveTime > 1000) { // 每秒更新一次
                float dx = playerX - m.x;
                float dz = playerZ - m.z;
                float dist = sqrtf(dx * dx + dz * dz);
                
                uint32_t rng = worldSeed ^ (uint32_t)m.x ^ (uint32_t)m.z ^ now;
                rng = rng * 1103515245 + 12345;
                
                if (dist < 12.0f && (rng & 0x3) == 0) {
                    // 30% 概率向玩家移动
                    float invDist = 1.0f / (dist + 0.1f);
                    m.vx = dx * invDist * 0.03f;
                    m.vz = dz * invDist * 0.03f;
                } else {
                    // 随机游走
                    float angle = ((float)(rng & 0xFF) / 255.0f) * 2.0f * MY_PI;
                    m.vx = cosf(angle) * 0.02f;
                    m.vz = sinf(angle) * 0.02f;
                }
                m.lastMoveTime = now;
            }
            
            // 应用移动
            float newX = m.x + m.vx;
            float newZ = m.z + m.vz;
            
            // 简单碰撞检测：检查新位置是否有方块
            int blockX = fastFloor(newX);
            int blockZ = fastFloor(newZ);
            int blockY = fastFloor(m.y);
            if (getBlockFast(blockX, blockY, blockZ) == BLOCK_AIR &&
                getBlockFast(blockX, blockY + 1, blockZ) == BLOCK_AIR) {
                m.x = newX;
                m.z = newZ;
            } else {
                // 碰撞，停止移动
                m.vx = 0;
                m.vz = 0;
            }
            
            // 重力
            if (getBlockFast(fastFloor(m.x), fastFloor(m.y - 0.5f), fastFloor(m.z)) == BLOCK_AIR) {
                m.y -= 0.02f;
                if (m.y < 1.0f) {
                    m.active = false; // 掉出世界
                }
            } else {
                // 站在方块上，稍微向上推
                m.y = (float)fastFloor(m.y - 0.5f) + 1.5f;
            }
        }
    }
    static int worldToChunkCoord(int w) { return w >= 0 ? w / CHUNK_SIZE_XZ : (w - CHUNK_SIZE_XZ + 1) / CHUNK_SIZE_XZ; }
    static int worldToLocalCoord(int w) { int local = w % CHUNK_SIZE_XZ; return local >= 0 ? local : local + CHUNK_SIZE_XZ; }

    Chunk *ensureChunk(int cx, int cz) {
        int idx = getChunkIndex(cx, cz);
        Chunk *chunk = &chunks[idx];
        if (chunk->loaded && chunk->cx == cx && chunk->cz == cz) return chunk;
        if (chunkTaskQueue == nullptr || chunkMutex == nullptr) {
            // 如果未初始化，使用同步方式（降级处理）
            chunk->clear();
            chunk->cx = cx;
            chunk->cz = cz;
            uint32_t t1 = micros();
            bool loaded = g_storage.loadChunk(chunk);
            uint32_t dtLoad = micros() - t1;
            if (dtLoad > g_prof_chunkLoadMaxUs) g_prof_chunkLoadMaxUs = dtLoad;
            if (!loaded) {
                uint32_t t2 = micros();
                chunk->generate(g_noise, worldSeed);
                uint32_t dtGen = micros() - t2;
                if (dtGen > g_prof_chunkGenMaxUs) g_prof_chunkGenMaxUs = dtGen;
            }
            chunk->loaded = true;
            return chunk;
        }
        xSemaphoreTake(chunkMutex, portMAX_DELAY);
        if (chunk->loaded && chunk->cx == cx && chunk->cz == cz) { xSemaphoreGive(chunkMutex); return chunk; }
        if (chunk->loaded && chunk->modified) {
            ChunkTask saveTask = {CHUNK_TASK_SAVE_QUEUE, chunk, 0, 0, 0};
            xQueueSend(chunkTaskQueue, &saveTask, 0);
            chunk->modified = false;
        }
        chunk->clear(); chunk->cx = cx; chunk->cz = cz; chunk->loaded = false;
        xSemaphoreGive(chunkMutex);
        ChunkTask task = {CHUNK_TASK_LOAD, chunk, cx, cz, worldSeed};
        xQueueSend(chunkTaskQueue, &task, 0);
        int waitCount = 0;
        while (!chunk->loaded && waitCount < 100) { vTaskDelay(1); waitCount++; }
        return chunk;
    }

    void saveAllChunks() {
        if (chunkTaskQueue == nullptr) {
            for (int i = 0; i < CHUNK_POOL_SIZE; i++)
                if (chunks[i].loaded && chunks[i].modified) g_storage.saveChunk(&chunks[i]);
            return;
        }
        for (int i = 0; i < CHUNK_POOL_SIZE; i++) {
            if (chunks[i].loaded && chunks[i].modified) {
                ChunkTask saveTask = {CHUNK_TASK_SAVE_QUEUE, &chunks[i], 0, 0, 0};
                xQueueSend(chunkTaskQueue, &saveTask, portMAX_DELAY);
            }
        }
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
#if ENABLE_FALLING_BLOCKS
        if (isFallingBlock(type) && getBlock(wx, wy - 1, wz) == BLOCK_AIR && wy > 0) {
            spawnFalling(wx, wy, wz, type);
            return;
        }
#endif
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
#if ENABLE_FALLING_BLOCKS
                if (b == BLOCK_GRAVEL && under == BLOCK_AIR) {
                    if (!spawnFalling(wx, y, wz, b)) break;
                    continue;
                }
#endif
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
    
    // 获取当前光源方向（随时间转动的太阳，类似极地太阳，不落下）
    static Vec3f getLightDirection() {
        // 太阳在天空中转动：高度角在 0.4-0.95 之间（始终在地平线以上，类似极地）
        // 方位角随时间转动（360度循环）
        uint32_t timeMs = millis();
        float timeCycle = (float)(timeMs % 120000) / 120000.0f; // 120秒一个完整周期（更慢的转动）
        float azimuth = timeCycle * 2.0f * MY_PI; // 方位角：0-2π
        
        // 高度角：在 0.4-0.95 之间缓慢变化（类似极地太阳的轨迹）
        // 使用两个周期叠加，让高度角变化更自然
        float elevation = 0.4f + 0.55f * (0.5f + 0.5f * sinf(timeCycle * 2.0f * MY_PI * 0.5f));
        
        // 转换为方向向量（从光源指向方块，注意 y 是向上的）
        float cosElev = cosf(elevation);
        return Vec3f(cosf(azimuth) * cosElev, sinf(elevation), sinf(azimuth) * cosElev);
    }
    
    // 计算天空光：根据光源方向，从方块位置向光源方向查找
    float getSkyLight(int wx, int wy, int wz) {
        Vec3f lightDir = getLightDirection();
        // 如果光源在下方，直接返回最低光照
        if (lightDir.y < 0.0f) return 0.3f;
        
        // 从方块位置向光源方向查找遮挡
        Vec3f start((float)wx + 0.5f, (float)wy + 0.5f, (float)wz + 0.5f);
        Vec3f dir = lightDir;
        // 归一化方向
        float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        if (len > 0.001f) {
            dir.x /= len; dir.y /= len; dir.z /= len;
        }
        
        // 简化的光线追踪：检查几个关键点
        float lightLevel = 1.0f;
        for (int step = 1; step <= 8; step++) {
            float dist = (float)step * 0.5f;
            Vec3f checkPos = start + dir * dist;
            int checkX = fastFloor(checkPos.x);
            int checkY = fastFloor(checkPos.y);
            int checkZ = fastFloor(checkPos.z);
            
            if (checkY < 0 || checkY >= CHUNK_SIZE_Y) break;
            if (getBlockFast(checkX, checkY, checkZ) != BLOCK_AIR) {
                // 被遮挡，根据距离衰减
                lightLevel = 1.0f - (float)step * 0.12f;
                break;
            }
        }
        
        // 根据光源高度角调整基础光照强度
        float baseIntensity = 0.3f + lightDir.y * 0.7f; // 高度角越高，光照越强
        return clampf(lightLevel * baseIntensity, 0.2f, 1.0f);
    }
    
    // 计算简易 AO（环境光遮蔽）：检查相邻方块的遮挡
    float getAO(int wx, int wy, int wz, int side) {
        float ao = 1.0f;
        // 根据面的方向，检查相邻的 3 个方块
        int dx1 = 0, dz1 = 0, dx2 = 0, dz2 = 0, dx3 = 0, dz3 = 0;
        int dy1 = 0, dy2 = 0, dy3 = 0;
        
        if (side == 0) { // 顶面
            dx1 = -1; dz1 = 0; dy1 = 0;
            dx2 = 0; dz2 = -1; dy2 = 0;
            dx3 = -1; dz3 = -1; dy3 = 0;
        } else if (side == 1) { // 底面
            dx1 = -1; dz1 = 0; dy1 = 0;
            dx2 = 0; dz2 = -1; dy2 = 0;
            dx3 = -1; dz3 = -1; dy3 = 0;
        } else if (side == 2) { // X 方向侧面
            dx1 = 0; dz1 = -1; dy1 = 0;
            dx2 = 0; dz2 = 0; dy2 = 1;
            dx3 = 0; dz3 = -1; dy3 = 1;
        } else { // Z 方向侧面
            dx1 = -1; dz1 = 0; dy1 = 0;
            dx2 = 0; dz2 = 0; dy2 = 1;
            dx3 = -1; dz3 = 0; dy3 = 1;
        }
        
        int occluded = 0;
        if (getBlockFast(wx + dx1, wy + dy1, wz + dz1) != BLOCK_AIR) occluded++;
        if (getBlockFast(wx + dx2, wy + dy2, wz + dz2) != BLOCK_AIR) occluded++;
        if (getBlockFast(wx + dx3, wy + dy3, wz + dz3) != BLOCK_AIR) occluded++;
        
        // 0 个遮挡 = 1.0, 1 个 = 0.8, 2 个 = 0.6, 3 个 = 0.4
        ao = 1.0f - (float)occluded * 0.2f;
        return ao;
    }
    RayHit raycast(Vec3f origin, Vec3f direction, float maxDist) {
        RayHit result; result.hit = false;
        float len2 = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (len2 < 1e-12f) return result;
        // 归一化判定放宽：减少 invSqrt 次数（渲染/射线热点）
        if (fabsf(len2 - 1.0f) > 5e-2f) {
            float invLen = fastInvSqrt(len2);
            direction.x *= invLen; direction.y *= invLen; direction.z *= invLen;
        }
        int mapX = fastFloor(origin.x), mapY = fastFloor(origin.y), mapZ = fastFloor(origin.z);

        // 更激进的“早退”：渲染里大量射线指向天空/上方时，没必要跑完整 MAX_RENDER_DIST
        // 交互射线通常 maxDist=5，不受影响
        if (maxDist > 12.0f && direction.y > 0.25f) maxDist = 12.0f;
        if (maxDist > 16.0f && direction.y > 0.40f) maxDist = 16.0f;
        // 使用位运算优化符号判断；距离步长改为定点为主，避免 DDA 循环中大量浮点加法
        float deltaDistXf = (direction.x == 0.0f) ? 1e6f : fabsf(1.0f / direction.x);
        float deltaDistYf = (direction.y == 0.0f) ? 1e6f : fabsf(1.0f / direction.y);
        float deltaDistZf = (direction.z == 0.0f) ? 1e6f : fabsf(1.0f / direction.z);
        fix16 deltaDistX = floatToFix16(deltaDistXf);
        fix16 deltaDistY = floatToFix16(deltaDistYf);
        fix16 deltaDistZ = floatToFix16(deltaDistZf);
        int stepX = (direction.x < 0.0f) ? -1 : 1;
        int stepY = (direction.y < 0.0f) ? -1 : 1;
        int stepZ = (direction.z < 0.0f) ? -1 : 1;
        float sideDistXf = (direction.x < 0.0f) ? (origin.x - (float)mapX) * deltaDistXf : ((float)(mapX + 1) - origin.x) * deltaDistXf;
        float sideDistYf = (direction.y < 0.0f) ? (origin.y - (float)mapY) * deltaDistYf : ((float)(mapY + 1) - origin.y) * deltaDistYf;
        float sideDistZf = (direction.z < 0.0f) ? (origin.z - (float)mapZ) * deltaDistZf : ((float)(mapZ + 1) - origin.z) * deltaDistZf;
        fix16 sideDistX = floatToFix16(sideDistXf);
        fix16 sideDistY = floatToFix16(sideDistYf);
        fix16 sideDistZ = floatToFix16(sideDistZf);
        fix16 maxDistFix = floatToFix16(maxDist);
        int side = 0, curCX = 0x7fffffff, curCZ = 0x7fffffff;
        Chunk *curChunk = nullptr;
        const int R = LOADED_RADIUS;
        for (int i = 0; i < MAX_RAY_STEPS; i++) {
            // 优化分支：使用条件移动减少分支预测失败
            if (sideDistX < sideDistY) {
                if (sideDistX < sideDistZ) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistZ += deltaDistZ;
                    mapZ += stepZ;
                    side = 2;
                }
            } else {
                if (sideDistY < sideDistZ) {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                } else {
                    sideDistZ += deltaDistZ;
                    mapZ += stepZ;
                    side = 2;
                }
            }
            if (mapY < 0 || mapY >= CHUNK_SIZE_Y) continue;
            fix16 distFix = (side == 0) ? (fix16)(sideDistX - deltaDistX)
                                        : (side == 1) ? (fix16)(sideDistY - deltaDistY)
                                                      : (fix16)(sideDistZ - deltaDistZ);
            if (distFix > maxDistFix) break;
            // 仅在加载半径内才取块，否则当 AIR（和你原来的 getBlockFast 行为一致）
            if (!centerValid) continue;

            // 缓存当前 chunk，避免每步都算 dx/dz（chunk 8 格才变一次）
            int cx = mapX >> 3;
            int cz = mapZ >> 3;
            if (cx != curCX || cz != curCZ) {
                curCX = cx; curCZ = cz;
                int dx = cx - centerCX, dz = cz - centerCZ;
                if ((unsigned)(dx + R) <= (unsigned)(2 * R) && (unsigned)(dz + R) <= (unsigned)(2 * R))
                    curChunk = grid[dx + R][dz + R];
                else curChunk = nullptr;
            }

            if (curChunk) {
                uint8_t b = curChunk->getBlockFast(mapX & 7, mapY, mapZ & 7);
                if (b != BLOCK_AIR) {
                    result.hit = true; result.distance = fix16ToFloat(distFix);
                    result.blockPos = Vec3i(mapX, mapY, mapZ);
                    result.block = (BlockType)b; result.normal = Vec3i(0, 0, 0);
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
#if !ENABLE_FALLING_BLOCKS
        return;
#else
        constexpr fix16 GRAV_FIX = (fix16)(GRAVITY_VAL * (float)(1 << FIX16_SHIFT));
        constexpr fix16 VY_MIN_FIX = (fix16)(-0.8f * (float)(1 << FIX16_SHIFT));
        constexpr fix16 HALF_FIX = (fix16)(0.5f * (float)(1 << FIX16_SHIFT));
        constexpr fix16 EPS_FIX = (fix16)(0.001f * (float)(1 << FIX16_SHIFT));
        for (int i = 0; i < MAX_FALLING; i++) {
            if (!falling[i].active) continue;
            falling[i].vy = (fix16)(falling[i].vy - GRAV_FIX);
            if (falling[i].vy < VY_MIN_FIX) falling[i].vy = VY_MIN_FIX;

            fix16 nextY = (fix16)(falling[i].y + falling[i].vy);
            int wx = fastFloor(falling[i].x), wz = fastFloor(falling[i].z);
            fix16 bottomNext = (fix16)(nextY - HALF_FIX);
            int supportY = fix16FloorToInt((fix16)(bottomNext - EPS_FIX));
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
#endif
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
    inline Vec3f getEyePosition() const { return Vec3f(position.x, position.y - P_HEIGHT + E_HEIGHT, position.z); }
    inline Vec3f getForward() const {
        float cp = g_fastMath.fastCos(pitch);
        float sy = g_fastMath.fastSin(yaw);
        float cy = g_fastMath.fastCos(yaw);
        float sp = g_fastMath.fastSin(pitch);
        return Vec3f(sy * cp, -sp, cy * cp);
    }
    bool checkCollision(float x, float y, float z) {
        const float hw = P_WIDTH * 0.5f; // 预计算
        int minX = fastFloor(x - hw);
        int maxX = fastFloor(x + hw);
        int minY = fastFloor(y - P_HEIGHT);
        int maxY = fastFloor(y);
        int minZ = fastFloor(z - hw);
        int maxZ = fastFloor(z + hw);
        // 提前检查边界，减少不必要的循环
        if (minY < 0 || maxY >= CHUNK_SIZE_Y) return true;
        for (int bx = minX; bx <= maxX; bx++) {
            for (int by = minY; by <= maxY; by++) {
                for (int bz = minZ; bz <= maxZ; bz++) {
                    if (g_world.getBlockFast(bx, by, bz) != BLOCK_AIR) return true;
                }
            }
        }
        return false;
    }
    void move(float dx, float dz) {
        float sy = g_fastMath.fastSin(yaw);
        float cy = g_fastMath.fastCos(yaw);
        // 直接计算，避免创建临时Vec3f对象
        float moveX = (sy * dz + cy * dx) * MOVE_SPEED;
        float moveZ = (cy * dz - sy * dx) * MOVE_SPEED;
        float newX = position.x + moveX;
        if (!checkCollision(newX, position.y, position.z)) {
            position.x = newX;
        }
        float newZ = position.z + moveZ;
        if (!checkCollision(position.x, position.y, newZ)) {
            position.z = newZ;
        }
    }
    void turn(float dyaw, float dpitch) {
        yaw += dyaw * TURN_SPEED;
        pitch += dpitch * LOOK_SPEED;
        if (pitch > 1.4f) pitch = 1.4f;
        else if (pitch < -1.4f) pitch = -1.4f;
        // 使用更快的角度归一化
        float twoPi = 2.0f * MY_PI;
        if (yaw > MY_PI) {
            yaw -= twoPi;
            if (yaw > MY_PI) yaw -= twoPi; // 处理极端情况
        } else if (yaw < -MY_PI) {
            yaw += twoPi;
            if (yaw < -MY_PI) yaw += twoPi; // 处理极端情况
        }
    }
    void jump() { if (onGround) { velocity.y = JUMP_VELOCITY; onGround = false; } }
    void update() {
        // Y 方向重力积分使用 16.16 定点，减少每帧浮点运算
        constexpr fix16 GRAV_FIX = (fix16)(GRAVITY_VAL * (float)(1 << FIX16_SHIFT));
        constexpr fix16 VY_MIN_FIX = (fix16)(-0.8f * (float)(1 << FIX16_SHIFT));
        fix16 vyFix = floatToFix16(velocity.y);
        vyFix = (fix16)(vyFix - GRAV_FIX);
        if (vyFix < VY_MIN_FIX) vyFix = VY_MIN_FIX;
        fix16 newYFix = (fix16)(floatToFix16(position.y) + vyFix);
        float newY = fix16ToFloat(newYFix);
        if (checkCollision(position.x, newY, position.z)) {
            if (velocity.y < 0) {
                float feetPos = newY - P_HEIGHT;
                int blockY = fastFloor(feetPos);
                position.y = (float)blockY + 1.0f + P_HEIGHT + 0.002f;
                velocity.y = 0; onGround = true;
            } else velocity.y = 0;
        } else {
            position.y = newY;
            velocity.y = fix16ToFloat(vyFix);
            onGround = false;
        }
        g_world.updateAroundPlayer(position.x, position.z);
        Vec3f eye = getEyePosition(), dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        hasTarget = hit.hit;
        if (hasTarget) targetPos = hit.blockPos;
    }
    bool breakBlock() {
        Vec3f eye = getEyePosition(), dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        if (hit.hit && hit.block != BLOCK_AIR && hit.block != BLOCK_BEDROCK) {
            g_world.setBlock(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z, BLOCK_AIR);
            return true;
        }
        return false;
    }
    bool placeBlock() {
        Vec3f eye = getEyePosition(), dir = getForward();
        RayHit hit = g_world.raycast(eye, dir, 5.0f);
        if (hit.hit) {
            int px = hit.blockPos.x + hit.normal.x, py = hit.blockPos.y + hit.normal.y, pz = hit.blockPos.z + hit.normal.z;
            BlockType old = g_world.getBlock(px, py, pz);
            g_world.setBlock(px, py, pz, selectedBlock);
            if (checkCollision(position.x, position.y, position.z)) { g_world.setBlock(px, py, pz, old); return false; }
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

    // Minecraft 风格纹理：基于 UV 坐标的程序化纹理（16x16 精细采样）
    inline uint16_t sampleMinecraftTexture(float u, float v, BlockType block, int side, uint16_t baseColor) {
        // 使用 16x16 网格（更精细的纹理细节）
        int ui = (int)(u * 8.0f) & 15;
        int vi = (int)(v * 8.0f) & 15;
        
        // 为每个方块类型生成不同的纹理模式
        uint32_t seed = (uint32_t)block * 1000 + side * 100 + ui * 17 + vi * 23;
        seed = seed * 1103515245 + 12345;
        
        // 提取基础颜色分量
        uint8_t r = ((baseColor >> 11) & 0x1F);
        uint8_t g = ((baseColor >> 5) & 0x3F);
        uint8_t b = (baseColor & 0x1F);
        
        // 根据方块类型应用不同的纹理效果
        float variation = 0.0f;
        switch (block) {
            case BLOCK_STONE:
            case BLOCK_COBBLESTONE:
                // 石头：随机点状纹理
                variation = ((seed & 0x3) == 0) ? -0.12f : ((seed & 0x7) == 0) ? 0.08f : 0.0f;
                break;
            case BLOCK_DIRT:
            case BLOCK_GRAVEL:
                // 泥土/砂砾：细密随机纹理
                variation = ((seed & 0x1) == 0) ? -0.08f : ((seed & 0x3) == 0) ? 0.06f : 0.0f;
                break;
            case BLOCK_GRASS:
                // 草地：顶部有随机草点，侧面保持泥土纹理
                if (side == 0) {
                    variation = ((seed & 0x5) == 0) ? 0.10f : ((seed & 0xF) == 0) ? -0.05f : 0.0f;
                } else {
                    variation = ((seed & 0x3) == 0) ? -0.06f : 0.0f;
                }
                break;
            case BLOCK_WOOD:
            case BLOCK_PLANK:
                // 木头：年轮/木纹效果
                if (side == 0 || side == 1) {
                    // 顶面/底面：年轮效果（基于距离中心的距离）
                    float centerU = fabsf(u - 0.5f), centerV = fabsf(v - 0.5f);
                    float distFromCenter = centerU + centerV;
                    variation = (distFromCenter < 0.2f) ? 0.05f : ((seed & 0x7) == 0) ? -0.04f : 0.0f;
                } else {
                    // 侧面：垂直木纹
                    variation = ((vi & 0x3) == 0) ? -0.06f : ((seed & 0x7) == 0) ? 0.04f : 0.0f;
                }
                break;
            case BLOCK_SAND:
                // 沙子：细密随机点
                variation = ((seed & 0x1) == 0) ? -0.05f : ((seed & 0x5) == 0) ? 0.04f : 0.0f;
                break;
            case BLOCK_BRICK:
                // 砖块：网格状纹理（模拟砖缝）
                if ((ui % 4 < 1) || (vi % 4 < 1)) {
                    variation = -0.15f; // 砖缝更暗
                } else {
                    variation = ((seed & 0x7) == 0) ? 0.05f : 0.0f;
                }
                break;
            case BLOCK_LEAVES:
                // 树叶：随机点状纹理
                variation = ((seed & 0x3) == 0) ? -0.08f : ((seed & 0x7) == 0) ? 0.06f : 0.0f;
                break;
            case BLOCK_IRON_ORE:
            case BLOCK_COAL_ORE:
                // 矿石：随机亮点
                variation = ((seed & 0x7) == 0) ? 0.12f : ((seed & 0xF) == 0) ? -0.08f : 0.0f;
                break;
            default:
                // 其他方块：轻微随机变化
                variation = ((seed & 0x7) == 0) ? -0.04f : 0.0f;
                break;
        }
        
        // 应用变化
        float factor = 1.0f + variation;
        r = (uint8_t)clampf((float)r * factor, 0.0f, 31.0f);
        g = (uint8_t)clampf((float)g * factor, 0.0f, 63.0f);
        b = (uint8_t)clampf((float)b * factor, 0.0f, 31.0f);
        
        return (r << 11) | (g << 5) | b;
    }
    
    inline uint16_t getBlockColor(BlockType block, int side, float dist, float u = 0.5f, float v = 0.5f, bool isSelected = false, float lightLevel = 1.0f) {
        uint16_t color;
        // 使用数组查找代替switch，减少分支
        static const uint16_t topColors[BLOCK_COUNT] = {
            0x0000, COLOR_STONE_TOP, COLOR_DIRT_TOP, COLOR_GRASS_TOP,
            COLOR_WOOD_TOP, COLOR_LEAVES_TOP, COLOR_BEDROCK_TOP, COLOR_GRAVEL_TOP,
            COLOR_SAND_TOP, COLOR_WATER_TOP, COLOR_COBBLE_TOP, COLOR_PLANK_TOP,
            COLOR_BRICK_TOP, COLOR_GLASS_TOP, COLOR_IRONORE_TOP, COLOR_COALORE_TOP
        };
        static const uint16_t sideColors[BLOCK_COUNT] = {
            0x0000, COLOR_STONE_SIDE, COLOR_DIRT_SIDE, COLOR_GRASS_SIDE,
            COLOR_WOOD_SIDE, COLOR_LEAVES_SIDE, COLOR_BEDROCK_SIDE, COLOR_GRAVEL_SIDE,
            COLOR_SAND_SIDE, COLOR_WATER_SIDE, COLOR_COBBLE_SIDE, COLOR_PLANK_SIDE,
            COLOR_BRICK_SIDE, COLOR_GLASS_SIDE, COLOR_IRONORE_SIDE, COLOR_COALORE_SIDE
        };
        if (block < BLOCK_COUNT) {
            color = (side == 0) ? topColors[block] : sideColors[block];
        } else {
            color = 0x0000;
        }
        
        // LOD：距离超过 9 格就不应用纹理和边界效果，直接返回纯色（减小半径）
        constexpr float TEXTURE_DISTANCE = 9.0f;
        bool isFar = (dist > TEXTURE_DISTANCE);
        
        if (isFar) {
            // 远距离：只应用光照，不应用纹理和边界
            uint8_t r = ((color >> 11) & 0x1F);
            uint8_t g = ((color >> 5) & 0x3F);
            uint8_t b = (color & 0x1F);
            float lightFactor = clampf(lightLevel, 0.2f, 1.0f);
            r = (uint8_t)((float)r * lightFactor);
            g = (uint8_t)((float)g * lightFactor);
            b = (uint8_t)((float)b * lightFactor);
            color = (r << 11) | (g << 5) | b;
            
            // 选中方块：远距离时也应用混合色
            if (isSelected) {
                r = ((color >> 11) & 0x1F);
                g = ((color >> 5) & 0x3F);
                b = (color & 0x1F);
                r = (uint8_t)((int)r * 0.6f + 0x1F * 0.4f);
                g = (uint8_t)((int)g * 0.6f + 0x3F * 0.4f);
                b = (uint8_t)((int)b * 0.6f + 0x1F * 0.4f);
                color = (r << 11) | (g << 5) | b;
            }
            return color;
        }
        
        // Minecraft 风格纹理：近距离应用精细纹理
        color = sampleMinecraftTexture(u, v, block, side, color);
        
        // 方块边界效果：在边缘区域变暗，让边界更明显（减小边界宽度，更精细）
        constexpr float BORDER_WIDTH = 0.06f;
        float edgeFactor = 1.0f;
        if (u < BORDER_WIDTH) edgeFactor = u / BORDER_WIDTH;
        else if (u > 1.0f - BORDER_WIDTH) edgeFactor = (1.0f - u) / BORDER_WIDTH;
        if (v < BORDER_WIDTH) {
            float vFactor = v / BORDER_WIDTH;
            if (vFactor < edgeFactor) edgeFactor = vFactor;
        } else if (v > 1.0f - BORDER_WIDTH) {
            float vFactor = (1.0f - v) / BORDER_WIDTH;
            if (vFactor < edgeFactor) edgeFactor = vFactor;
        }
        
        // 应用边界变暗
        if (edgeFactor < 1.0f) {
            uint8_t r = ((color >> 11) & 0x1F);
            uint8_t g = ((color >> 5) & 0x3F);
            uint8_t b = (color & 0x1F);
            float darken = 0.4f + 0.6f * edgeFactor; // 边缘 0.4，中心 1.0
            r = (uint8_t)((float)r * darken);
            g = (uint8_t)((float)g * darken);
            b = (uint8_t)((float)b * darken);
            color = (r << 11) | (g << 5) | b;
        }
        
        // 应用光照（天空光 + AO）
        uint8_t r = ((color >> 11) & 0x1F);
        uint8_t g = ((color >> 5) & 0x3F);
        uint8_t b = (color & 0x1F);
        // lightLevel: 0.0 (完全黑暗) -> 1.0 (完全明亮)
        float lightFactor = clampf(lightLevel, 0.2f, 1.0f); // 最低 20% 亮度（环境光）
        r = (uint8_t)((float)r * lightFactor);
        g = (uint8_t)((float)g * lightFactor);
        b = (uint8_t)((float)b * lightFactor);
        color = (r << 11) | (g << 5) | b;
        
        // 选中方块：混合白色（60% 原色 + 40% 白色）
        if (isSelected) {
            r = ((color >> 11) & 0x1F);
            g = ((color >> 5) & 0x3F);
            b = (color & 0x1F);
            r = (uint8_t)((int)r * 0.6f + 0x1F * 0.4f);
            g = (uint8_t)((int)g * 0.6f + 0x3F * 0.4f);
            b = (uint8_t)((int)b * 0.6f + 0x1F * 0.4f);
            color = (r << 11) | (g << 5) | b;
        }
        return color;
    }
public:
    void init() {
        M5Cardputer.Display.setRotation(1);
        M5Cardputer.Display.setSwapBytes(true);
        M5Cardputer.Display.fillScreen(COLOR_SKY);
    }
    inline void drawCrosshairToBuffer() {
        const uint16_t c = 0xFFFF;
        const int cx = SCREEN_CENTER_X, cy = SCREEN_CENTER_Y;
        int x0 = cx - 5, x1 = cx + 5;
        if (x0 < 0) x0 = 0; if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
        for (int x = x0; x <= x1; x++) frameBuffer[cy * SCREEN_W + x] = c;
        int y0 = cy - 5, y1 = cy + 5;
        if (y0 < 0) y0 = 0; if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
        for (int y = y0; y <= y1; y++) frameBuffer[y * SCREEN_W + cx] = c;
    }
    void setRenderStep(uint8_t s) { if (s < 1) s = 1; if (s > 4) s = 4; renderStep = s; }
    uint8_t getRenderStep() const { return renderStep; }
    void render() {
        Vec3f eye = g_player.getEyePosition();
        float yaw = g_player.yaw, pitch = g_player.pitch;
        float cosYaw = g_fastMath.fastCos(yaw), sinYaw = g_fastMath.fastSin(yaw);
        float cosPitch = g_fastMath.fastCos(pitch), sinPitch = g_fastMath.fastSin(pitch);
        const int STEP = (int)renderStep;
        const float xScale = TAN_HALF_FOV * ASPECT_RATIO;
        const float yScale = TAN_HALF_FOV;
        const float xStep = xScale * ((float)STEP * INV_CENTER_X);

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
            float yNorm = ((float)SCREEN_CENTER_Y - (float)screenY) * INV_CENTER_Y, yVal = yNorm * yScale, y2 = yVal * yVal;
            float baseRowX = yVal * colYx + colZx, baseRowY = yVal * colYy + colZy, baseRowZ = yVal * colYz + colZz;
            float xVal = -xScale, worldBaseX = xVal * colXx + baseRowX, worldBaseY = baseRowY, worldBaseZ = xVal * colXz + baseRowZ;
            for (int screenX = 0; screenX < SCREEN_W; screenX += STEP) {
                float lenSq = xVal * xVal + y2 + 1.0f;
                float invLen = fastInvSqrt(lenSq);
                Vec3f rayDir(worldBaseX * invLen, worldBaseY * invLen, worldBaseZ * invLen);
                RayHit hit = g_world.raycast(eye, rayDir, (float)MAX_RENDER_DIST);
                uint16_t color;
                if (hit.hit) {
                    int side = (hit.normal.y > 0) ? 0 : (hit.normal.y < 0) ? 1 : (hit.normal.x != 0) ? 2 : 3;
                    Vec3f hitPos = eye + rayDir * hit.distance;
                    float u = 0, v = 0;
                    if (side == 0 || side == 1) { u = hitPos.x - fastFloor(hitPos.x); v = hitPos.z - fastFloor(hitPos.z); }
                    else if (side == 2) { u = hitPos.y - fastFloor(hitPos.y); v = hitPos.z - fastFloor(hitPos.z); }
                    else { u = hitPos.x - fastFloor(hitPos.x); v = hitPos.y - fastFloor(hitPos.y); }
                    bool isSelected = (g_player.hasTarget && hit.blockPos == g_player.targetPos);
                    // 计算光照：天空光 + AO
                    float skyLight = g_world.getSkyLight(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z);
                    float ao = g_world.getAO(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z, side);
                    float lightLevel = skyLight * ao; // 天空光乘以 AO
                    color = getBlockColor(hit.block, side, hit.distance, u, v, isSelected, lightLevel);
                } else color = COLOR_SKY;
                int h = (screenY + STEP <= SCREEN_H) ? STEP : (SCREEN_H - screenY);
                int w = (screenX + STEP <= SCREEN_W) ? STEP : (SCREEN_W - screenX);
                uint16_t *fbPtr = &frameBuffer[screenY * SCREEN_W + screenX];
                for (int dy = 0; dy < h; ++dy) {
                    uint16_t *row = fbPtr + dy * SCREEN_W;
                    for (int dxp = 0; dxp < w; ++dxp) row[dxp] = color;
                }
                worldBaseX += dWx; worldBaseY += dWy; worldBaseZ += dWz; xVal += xStep;
            }
        }
#if ENABLE_FALLING_BLOCKS
        for (int i = 0; i < World::getMaxFallingEnts(); i++) {
            const World::FallingEnt *e = &g_world.getFallingEnts()[i];
            if (!e->active) continue;
            float ex = e->x;
            float ey = fix16ToFloat(e->y);
            float ez = e->z;
            float rx = ex - eye.x, ry = ey - eye.y, rz = ez - eye.z;
            float cx = rx * cosYaw - rz * sinYaw, cz = rx * sinYaw + rz * cosYaw;
            float cy = ry * cosPitch - cz * sinPitch, cz2 = ry * sinPitch + cz * cosPitch;
            if (cz2 <= 0.2f) continue;
            float invCz2 = 1.0f / cz2;
            float sxNorm = (cx * invCz2) / (TAN_HALF_FOV * ASPECT_RATIO), syNorm = (cy * invCz2) / TAN_HALF_FOV;
            int sx = SCREEN_CENTER_X + (int)(sxNorm * SCREEN_CENTER_X), sy = SCREEN_CENTER_Y - (int)(syNorm * SCREEN_CENTER_Y);
            int size = (int)((SCREEN_H * 0.7f) / cz2);
            if (size < 2) size = 2; else if (size > 18) size = 18;
            uint16_t col = getBlockColor(e->type, 0, cz2);
            int half = size >> 1, x0 = sx - half, x1 = sx + half, y0 = sy - half, y1 = sy + half;
            if (x1 < 0 || x0 >= SCREEN_W || y1 < 0 || y0 >= SCREEN_H) continue;
            if (x0 < 0) x0 = 0; if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
            if (y0 < 0) y0 = 0; if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
            uint16_t *fbBase = frameBuffer;
            for (int yy = y0; yy <= y1; yy++) {
                uint16_t *row = fbBase + yy * SCREEN_W + x0;
                for (int xx = x0; xx <= x1; xx++) *row++ = col;
            }
        }
#endif
        // 渲染生物（2D 简单绘制，不使用3D投影）
        for (int i = 0; i < World::getMaxMobs(); i++) {
            const World::Mob *m = &g_world.getMobs()[i];
            if (!m->active) continue;
            
            // 计算生物相对于玩家的2D位置（俯视图）
            float dx = m->x - g_player.position.x;
            float dz = m->z - g_player.position.z;
            float dy = m->y - g_player.position.y; // 高度差
            float dist2D = sqrtf(dx * dx + dz * dz);
            
            // 只显示附近的生物（12格内）
            if (dist2D > 12.0f || dist2D < 0.5f) continue;
            
            // 转换为屏幕坐标（简单的2D映射，考虑玩家朝向）
            // 将世界坐标旋转到玩家视角（XZ平面）
            float relX = dx * cosYaw + dz * sinYaw;
            float relZ = -dx * sinYaw + dz * cosYaw;
            
            // 映射到屏幕中心区域（简单的正交投影）
            // relX 对应屏幕 X，relZ 对应屏幕 Y（前后）
            int sx = SCREEN_CENTER_X + (int)(relX * 6.0f); // 缩放因子
            int sy = SCREEN_CENTER_Y + (int)(relZ * 6.0f) - (int)(dy * 3.0f); // 考虑高度差
            
            // 简单的生物图标：小方块
            const int mobSize = 3;
            int x0 = sx - mobSize, x1 = sx + mobSize;
            int y0 = sy - mobSize, y1 = sy + mobSize;
            
            if (x1 < 0 || x0 >= SCREEN_W || y1 < 0 || y0 >= SCREEN_H) continue;
            if (x0 < 0) x0 = 0; if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
            if (y0 < 0) y0 = 0; if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
            
            uint16_t mobColor = 0x07E0; // 绿色
            uint16_t *fbBase = frameBuffer;
            for (int yy = y0; yy <= y1; yy++) {
                uint16_t *row = fbBase + yy * SCREEN_W + x0;
                for (int xx = x0; xx <= x1; xx++) {
                    row[xx - x0] = mobColor;
                }
            }
        }
        drawCrosshairToBuffer(); // Step4-1
        M5Cardputer.Display.pushImage(0, 0, SCREEN_W, SCREEN_H, frameBuffer);
    }
    void drawHUD(int fpsVal) {
        static uint8_t tick = 0;
        static char posBuf[32], blkBuf[16], fpsBuf[16];
        if (tick == 0) {
            snprintf(posBuf, sizeof(posBuf), "%.1f %.1f %.1f", g_player.position.x, g_player.position.y, g_player.position.z);
            const char *blockNames[] = {"AIR","STONE","DIRT","GRASS","WOOD","LEAVES","BEDROCK","GRAVEL","SAND","WATER","COBBLE","PLANK","BRICK","GLASS","IRON","COAL"};
            snprintf(blkBuf, sizeof(blkBuf), "[%s]", blockNames[g_player.selectedBlock]);
            snprintf(fpsBuf, sizeof(fpsBuf), "FPS:%d", fpsVal);
        }
        tick = (tick + 1) % 5;
        M5Cardputer.Display.startWrite();
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(0xFFFF, COLOR_SKY);
        M5Cardputer.Display.setCursor(2, 2);
        M5Cardputer.Display.print(fpsBuf); M5Cardputer.Display.print("   ");
        M5Cardputer.Display.setCursor(SCREEN_W - 60, 2);
        M5Cardputer.Display.print(blkBuf); M5Cardputer.Display.print("   ");
        M5Cardputer.Display.setCursor(2, SCREEN_H - 10);
        M5Cardputer.Display.print(posBuf); M5Cardputer.Display.print("   ");
        M5Cardputer.Display.endWrite();
        // 物品栏功能暂时禁用
        // g_inventory.drawOverlay();
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
    bool delArmed = false;
    uint32_t delArmMs = 0;

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
        delArmed = false; delArmMs = 0;
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
        if (slotHasSave && delArmed && (millis() - delArmMs) < 1500) {
            M5Cardputer.Display.setTextColor(0xF800); // red
            M5Cardputer.Display.setCursor(5, 115);
            M5Cardputer.Display.print("FN+DEL again to DELETE save!");
        }
    }

    void drawPauseMenu() {
        for (int y = 0; y < SCREEN_H; y += 2)
            for (int x = 0; x < SCREEN_W; x += 2)
                M5Cardputer.Display.drawPixel(x + (y / 2) % 2, y, 0x0000);
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
            // Delete save: Fn+Del twice within 1.5s
            if (slotHasSave && keys.fn && keys.del) {
                uint32_t now = millis();
                if (!delArmed || (now - delArmMs) > 1500) {
                    delArmed = true;
                    delArmMs = now;
                    return false;
                } else {
                    delArmed = false;
                    g_storage.deleteWorldSlot(currentWorldSlot);
                    checkSlot();
                    if (!slotHasSave) randomizeSeed();
                    return false;
                }
            }
            if (delArmed && (millis() - delArmMs) > 1500) delArmed = false;
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
constexpr int8_t PerlinNoise::grad2[8][2];
PerlinNoise g_noise; World g_world; Player g_player; Renderer g_renderer; UI g_ui; Storage g_storage; WarpDB g_warps; CommandConsole g_console;
FastMath g_fastMath;
TaskHandle_t backgroundTaskHandle = nullptr;
volatile uint32_t g_prof_chunkSaveMaxUs = 0, g_prof_chunkLoadMaxUs = 0, g_prof_chunkGenMaxUs = 0;

// 简单帧率统计（不做任何限速，只用于 HUD 显示）
static unsigned long g_lastFpsTime = 0;
static int g_frameCount = 0;
static int g_fps = 0;

Inventory g_inventory;

void handleGameInput() {
    M5Cardputer.update();
    static bool lastEnterKey = false, lastPlaceKey = false;
    Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();

    // 物品栏功能暂时禁用
    // for (size_t i = 0; i < keys.word.size(); ++i) {
    //     char c = keys.word[i];
    //     if (c == 'i' || c == 'I') {
    //         g_inventory.toggleOpen();
    //         if (g_inventory.isOpen()) return;
    //     }
    // }
    // if (g_inventory.isOpen()) {
    //     if (g_inventory.handleInput(keys)) {
    //         g_player.selectedBlock = g_inventory.getSelectedBlock();
    //     }
    //     return;
    // }
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
            // 数字键直接选方块 (Fn+数字 = 调整渲染步长)
            case '1': if (keys.fn) g_renderer.setRenderStep(1); else g_player.selectedBlock = BLOCK_DIRT; break;
            case '2': if (keys.fn) g_renderer.setRenderStep(2); else g_player.selectedBlock = BLOCK_STONE; break;
            case '3': if (keys.fn) g_renderer.setRenderStep(3); else g_player.selectedBlock = BLOCK_GRASS; break;
            case '4': if (keys.fn) g_renderer.setRenderStep(4); else g_player.selectedBlock = BLOCK_WOOD; break;
            case '5': g_player.selectedBlock = BLOCK_SAND; break;
            case '6': g_player.selectedBlock = BLOCK_COBBLESTONE; break;
            case '7': g_player.selectedBlock = BLOCK_PLANK; break;
            case '8': g_player.selectedBlock = BLOCK_BRICK; break;
            case '9': g_player.selectedBlock = BLOCK_GLASS; break;
            case '0': g_player.selectedBlock = BLOCK_WATER; break;
            case 't': case 'T': if (keys.fn) { g_console.open(); return; } break;
        }
    }
    if (keys.enter && !lastEnterKey) g_player.breakBlock();
    lastEnterKey = keys.enter;
    if (placeKeyPressed && !lastPlaceKey) g_player.placeBlock();
    lastPlaceKey = placeKeyPressed;
    if (dx != 0 || dz != 0) g_player.move(dx, dz);
}

void backgroundTask(void *pvParameters) {
    while (true) {
        if (g_world.chunkTaskQueue == nullptr) { vTaskDelay(10); continue; }
        ChunkTask task;
        while (xQueueReceive(g_world.chunkTaskQueue, &task, 0) == pdTRUE) {
            if (task.type == CHUNK_TASK_LOAD) {
                if (g_world.chunkMutex == nullptr) continue;
                xSemaphoreTake(g_world.chunkMutex, portMAX_DELAY);
                if (!task.chunk->loaded) {
                    uint32_t t1 = micros();
                    bool loaded = g_storage.loadChunk(task.chunk);
                    uint32_t dtLoad = micros() - t1;
                    if (dtLoad > g_prof_chunkLoadMaxUs) g_prof_chunkLoadMaxUs = dtLoad;
                    if (!loaded) {
                        uint32_t t2 = micros();
                        task.chunk->generate(g_noise, task.worldSeed);
                        uint32_t dtGen = micros() - t2;
                        if (dtGen > g_prof_chunkGenMaxUs) g_prof_chunkGenMaxUs = dtGen;
                    }
                    task.chunk->loaded = true;
                }
                xSemaphoreGive(g_world.chunkMutex);
            } else if (task.type == CHUNK_TASK_SAVE_QUEUE) {
                g_storage.queueChunkSave(task.chunk);
            }
        }
        g_storage.processSaveQueue(4);
        vTaskDelay(1);
    }
}

void setup() {
    auto cfg = M5.config(); M5Cardputer.begin(cfg);
    M5Cardputer.Display.setRotation(1); M5Cardputer.Display.setBrightness(80); M5Cardputer.Display.fillScreen(0x0000);
    g_fastMath.init(); g_storage.init(); g_ui.init(); g_renderer.init();
    g_lastFpsTime = millis();
    g_frameCount = 0;
    g_fps = 0;
    (void)xTaskCreatePinnedToCore(backgroundTask, "ChunkIO", 4096, nullptr, 1, &backgroundTaskHandle, 0);
}

void startGame(uint32_t seedInput) {
    int slot = g_ui.getSelectedSlot(); g_storage.setWorldSlot(slot); g_warps.load();
    uint32_t finalSeed = g_storage.hasSave(slot) ? g_storage.loadWorldSeed() : seedInput;
    if (!g_storage.hasSave(slot)) g_storage.saveWorldSeed(finalSeed);
    g_world.init(finalSeed); g_player.init(finalSeed); g_player.load();
    // 物品栏功能暂时禁用
    // g_inventory.initDefault();
    // g_player.selectedBlock = g_inventory.getSelectedBlock();
    g_world.updateAroundPlayer(g_player.position.x, g_player.position.z);
    g_ui.state = STATE_PLAYING;
}

void loop() {
    // 帧率统计：仅计数和取样，不做限速
    g_frameCount++;
    unsigned long now = millis();
    if (now - g_lastFpsTime >= 1000) {
        g_fps = g_frameCount;
        g_frameCount = 0;
        g_lastFpsTime = now;
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
            // 生物系统：生成和更新
            g_world.spawnMobsNearPlayer(g_player.position.x, g_player.position.y, g_player.position.z);
            g_world.updateMobs(g_player.position.x, g_player.position.y, g_player.position.z);
            g_renderer.render(); g_renderer.drawHUD(g_fps);
            if (g_console.active) g_console.showList ? g_console.drawList() : g_console.drawBar();
            break;
        case STATE_PAUSED:
            g_ui.drawPauseMenu();
            int result = g_ui.handlePauseInput();
            if (result == 1) g_ui.state = STATE_PLAYING;
            else if (result == 2) {
                g_player.save(); g_world.saveAllChunks();
                if (g_world.chunkTaskQueue != nullptr)
                    while (uxQueueMessagesWaiting(g_world.chunkTaskQueue) > 0) vTaskDelay(10);
                g_storage.flushSaveQueue();
                g_ui.state = STATE_MENU; g_ui.init();
            }
            break;
    }
}
