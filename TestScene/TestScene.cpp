#include "raylib-cpp.hpp"
#include "rlgl.h"

#include "gekko_physics.h"

// ── Helpers ──────────────────────────────────────────────────────────

static Vector3 ToRL(const GekkoMath::Vec3F& v) { return { v.x, v.y, v.z }; }



// ── Raylib DebugDraw implementation ─────────────────────────────────

class RaylibDebugDraw : public GekkoPhysics::DebugDraw {
    using Vec3F = GekkoMath::Vec3F;
    using Mat3F = GekkoMath::Mat3F;

    // Renderer-chosen colors (high contrast on white)
    static constexpr ::Color COL_SHAPE      = { 30, 60, 180, 255 };    // strong blue
    static constexpr ::Color COL_SHAPE_FILL = { 30, 60, 180, 50 };
    static constexpr ::Color COL_AABB       = { 0, 160, 0, 255 };      // green
    static constexpr ::Color COL_CONTACT    = { 220, 0, 0, 255 };      // red
    static constexpr ::Color COL_NORMAL     = { 220, 0, 0, 255 };      // red (same as contact)
    static constexpr ::Color COL_LINE       = { 40, 40, 40, 255 };     // near-black

public:
    void DrawSphere(const Vec3F& center, float radius) override {
        DrawSphereEx(ToRL(center), radius, 12, 12, COL_SHAPE_FILL);
        DrawSphereWires(ToRL(center), radius, 12, 12, COL_SHAPE);
    }

    void DrawBox(const Vec3F& center, const Vec3F& half_extents, const Mat3F& rot) override {
        Vector3 size = { half_extents.x * 2, half_extents.y * 2, half_extents.z * 2 };

        rlPushMatrix();
        rlTranslatef(center.x, center.y, center.z);
        float m[16] = {
            rot.cols[0].x, rot.cols[0].y, rot.cols[0].z, 0,
            rot.cols[1].x, rot.cols[1].y, rot.cols[1].z, 0,
            rot.cols[2].x, rot.cols[2].y, rot.cols[2].z, 0,
            0, 0, 0, 1,
        };
        rlMultMatrixf(m);
        DrawCubeV({ 0, 0, 0 }, size, COL_SHAPE_FILL);
        DrawCubeWiresV({ 0, 0, 0 }, size, COL_SHAPE);
        rlPopMatrix();
    }

    void DrawCapsule(const Vec3F& start, const Vec3F& end, float radius) override {
        ::DrawCapsule(ToRL(start), ToRL(end), radius, 8, 8, COL_SHAPE_FILL);
        DrawCapsuleWires(ToRL(start), ToRL(end), radius, 8, 8, COL_SHAPE);
    }

    void DrawAABB(const Vec3F& min, const Vec3F& max) override {
        const float r = 0.02f;
        Vector3 v[8] = {
            {min.x, min.y, min.z}, {max.x, min.y, min.z},
            {max.x, min.y, max.z}, {min.x, min.y, max.z},
            {min.x, max.y, min.z}, {max.x, max.y, min.z},
            {max.x, max.y, max.z}, {min.x, max.y, max.z},
        };
        int edges[][2] = {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7},
        };
        for (auto& e : edges) {
            DrawCylinderEx(v[e[0]], v[e[1]], r, r, 4, COL_AABB);
        }
    }

    void DrawLine(const Vec3F& from, const Vec3F& to) override {
        DrawCylinderEx(ToRL(from), ToRL(to), 0.05f, 0.05f, 4, COL_NORMAL);
    }

    void DrawPoint(const Vec3F& position, float size) override {
        DrawSphereEx(ToRL(position), 0.2f, 6, 6, COL_CONTACT);
    }

    void DrawBodyOrigin(const Vec3F& position) override {
        static constexpr ::Color COL_ORIGIN = { 140, 40, 200, 255 }; // purple
        DrawSphereEx(ToRL(position), 0.12f, 6, 6, COL_ORIGIN);
    }

    void DrawBodyAxes(const Vec3F& position, const Mat3F& rot) override {
        Vector3 pos = ToRL(position);
        Vector3 x = { pos.x + rot.cols[0].x, pos.y + rot.cols[0].y, pos.z + rot.cols[0].z };
        Vector3 y = { pos.x + rot.cols[1].x, pos.y + rot.cols[1].y, pos.z + rot.cols[1].z };
        Vector3 z = { pos.x + rot.cols[2].x, pos.y + rot.cols[2].y, pos.z + rot.cols[2].z };
        DrawCylinderEx(pos, x, 0.03f, 0.03f, 4, {220, 40, 40, 255});   // X = red
        DrawCylinderEx(pos, y, 0.03f, 0.03f, 4, {40, 180, 40, 255});   // Y = green
        DrawCylinderEx(pos, z, 0.03f, 0.03f, 4, {40, 80, 220, 255});   // Z = blue
    }
};

// ── Scene setup ─────────────────────────────────────────────────────

static GekkoPhysics::Identifier BuildScene(GekkoPhysics::World& world) {
    using namespace GekkoPhysics;
    using namespace GekkoMath;

    // Helper: create body + shape group
    auto makeBody = [&](Vec3 pos, bool is_static, Mat3 rot = Mat3()) {
        Identifier bid = world.CreateBody();
        Body& b = world.GetBody(bid);
        b.position = pos;
        b.is_static = is_static;
        b.rotation = rot;
        Identifier gid = world.AddShapeGroup(bid);
        ShapeGroup& sg = world.GetShapeGroup(gid);
        sg.layer = 1; sg.mask = 0xFFFFFFFF;
        return std::pair<Identifier, Identifier>{bid, gid};
    };

    auto addSphereShape = [&](Identifier gid, Vec3 center, Unit radius) {
        Identifier sid = world.AddShape(gid, Shape::Sphere);
        Sphere& s = world.GetSphere(world.GetShape(sid).shape_type_id);
        s.center = center;
        s.radius = radius;
    };

    auto addOBBShape = [&](Identifier gid, Vec3 center, Vec3 half, Mat3 rot = Mat3()) {
        Identifier sid = world.AddShape(gid, Shape::OBB);
        OBB& o = world.GetOBB(world.GetShape(sid).shape_type_id);
        o.center = center;
        o.half_extents = half;
        o.rotation = rot;
    };

    auto addCapsuleShape = [&](Identifier gid, Vec3 start, Vec3 end, Unit radius) {
        Identifier sid = world.AddShape(gid, Shape::Capsule);
        Capsule& c = world.GetCapsule(world.GetShape(sid).shape_type_id);
        c.start = start;
        c.end = end;
        c.radius = radius;
    };

    Vec3 zero;
    Vec3 up(Unit{0}, Unit{1}, Unit{0});
    Vec3 down(Unit{0}, Unit{-1}, Unit{0});

    // Controlled body: static sphere you move with arrow keys
    Identifier controlled_body;
    {
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{1}, Unit{0}), false);
        addSphereShape(g, Vec3(Unit{-4}, Unit{0}, Unit{0}), Unit{1});
        controlled_body = b;
    }

    // Dynamic single-shape targets
    {
        auto [b, g] = makeBody(Vec3(Unit{-8}, Unit{1}, Unit{0}), false);
        addSphereShape(g, zero, Unit{1});
    }
    {
        // Tall box
        auto [b, g] = makeBody(Vec3(Unit{8}, Unit{2}, Unit{0}), false);
        addOBBShape(g, zero, Vec3(Unit{1}, Unit{2}, Unit{1}));
    }
    {
        // Diagonal capsule
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{1}, Unit{-8}), false);
        addCapsuleShape(g, Vec3(Unit{-1}, Unit{-1}, Unit{0}), Vec3(Unit{1}, Unit{1}, Unit{0}), Unit{1} / Unit{2});
    }
    {
        // Rotated wide box
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{1}, Unit{8}), false, Mat3::RotateY(45));
        addOBBShape(g, zero, Vec3(Unit{2}, Unit{1}, Unit{1}));
    }

    // Dynamic multi-shape bodies
    {
        // Sphere + tall OBB side by side
        auto [b, g] = makeBody(Vec3(Unit{-6}, Unit{1}, Unit{-6}), false);
        addSphereShape(g, Vec3(Unit{-1}, Unit{0}, Unit{0}), Unit{1});
        addOBBShape(g, Vec3(Unit{2}, Unit{0}, Unit{0}), Vec3(Unit{1}, Unit{2}, Unit{1}));
    }
    {
        // Tilted capsule + sphere
        auto [b, g] = makeBody(Vec3(Unit{6}, Unit{1}, Unit{-6}), false);
        addCapsuleShape(g, Vec3(Unit{0}, Unit{-1}, Unit{-1}), Vec3(Unit{0}, Unit{1}, Unit{1}), Unit{1} / Unit{2});
        addSphereShape(g, Vec3(Unit{2}, Unit{0}, Unit{0}), Unit{1});
    }
    {
        // Three spheres in a line
        auto [b, g] = makeBody(Vec3(Unit{-6}, Unit{1}, Unit{6}), false);
        addSphereShape(g, Vec3(Unit{-2}, Unit{0}, Unit{0}), Unit{1});
        addSphereShape(g, Vec3(Unit{0}, Unit{0}, Unit{0}), Unit{1});
        addSphereShape(g, Vec3(Unit{2}, Unit{0}, Unit{0}), Unit{1});
    }
    {
        // Tall OBB + angled capsule
        auto [b, g] = makeBody(Vec3(Unit{6}, Unit{1}, Unit{6}), false);
        addOBBShape(g, Vec3(Unit{0}, Unit{0}, Unit{-1}), Vec3(Unit{1}, Unit{3}, Unit{1}));
        addCapsuleShape(g, Vec3(Unit{-1}, Unit{0}, Unit{1}), Vec3(Unit{1}, Unit{1}, Unit{2}), Unit{1} / Unit{2});
    }

    return controlled_body;
}

// ── Main ────────────────────────────────────────────────────────────

int main() {
    raylib::Window window(1600, 900, "GekkoPhysics Test Scene");

    Camera3D camera = {};
    camera.position = { 10.0f, 8.0f, 10.0f };
    camera.target   = { 0.0f, 1.0f, 0.0f };
    camera.up       = { 0.0f, 1.0f, 0.0f };
    camera.fovy     = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    GekkoPhysics::World world;
    auto controlled = BuildScene(world);

    RaylibDebugDraw debug_draw;
    debug_draw.flags = GekkoPhysics::DrawFlag_All;
    world.SetDebugDraw(&debug_draw);

    const GekkoMath::Unit move_speed{1};
    SetTargetFPS(60);

    while (!window.ShouldClose()) {
        UpdateCamera(&camera, CAMERA_FIRST_PERSON);

        // Arrow keys move the controlled body on XZ plane
        {
            auto& body = world.GetBody(controlled);
            body.rotation *= GekkoMath::Mat3::RotateZ(1);
            if (IsKeyDown(KEY_RIGHT)) body.position.x += move_speed / GekkoMath::Unit{10};
            if (IsKeyDown(KEY_LEFT))  body.position.x -= move_speed / GekkoMath::Unit{10};
            if (IsKeyDown(KEY_UP))    body.position.z -= move_speed / GekkoMath::Unit{10};
            if (IsKeyDown(KEY_DOWN))  body.position.z += move_speed / GekkoMath::Unit{10};
        }

        world.Update();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

        // Pass 1: shapes + AABBs (with depth testing)
        debug_draw.flags = GekkoPhysics::DrawFlag_Shapes | GekkoPhysics::DrawFlag_AABBs;
        world.DrawDebug();

        // Pass 2: overlays (no depth testing — always on top)
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        debug_draw.flags = GekkoPhysics::DrawFlag_Contacts | GekkoPhysics::DrawFlag_BodyOrigins | GekkoPhysics::DrawFlag_BodyAxes;
        world.DrawDebug();
        rlDrawRenderBatchActive();
        rlEnableDepthTest();

        EndMode3D();

        DrawText("GekkoPhysics - Sphere | OBB | Capsule", 10, 10, 20, DARKGRAY);
        DrawText(TextFormat("Contacts: %d", (int)world.GetContacts().size()), 10, 35, 20, RED);
        auto& contacts = world.GetContacts();
        for (int i = 0; i < (int)contacts.size() && i < 4; i++) {
            auto p = contacts[i].point.AsFloat();
            DrawText(TextFormat("  C%d: (%.2f, %.2f, %.2f)", i, p.x, p.y, p.z), 10, 60 + i * 25, 20, RED);
        }
        DrawFPS(1920 - 100, 10);

        EndDrawing();
    }

    return 0;
}
