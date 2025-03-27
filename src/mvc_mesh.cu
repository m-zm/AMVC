// followinng mvc_mesh.cpp

#include <vector_types.h>
#include <vector_functions.h>

#include <chrono>
#include <string>

#include "mesh_io.hpp"

constexpr float pi = 3.14159265358979323846;
constexpr float eps = 1e-6;

constexpr int BLOCK_SIZE = 256;

struct Face_Vertices
{
    float3 v0;
    float3 v1;
    float3 v2;
};

__device__ float3 operator+(const float3 &a, const float3 &b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ float3 operator-(const float3 &a, const float3 &b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float3 operator*(const float3 &a, const float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}

__device__ float3 operator*(const float a, const float3 &b)
{
    return make_float3(a * b.x, a * b.y, a * b.z);
}

__device__ float3 operator/(const float3 &a, const float b)
{
    return make_float3(a.x / b, a.y / b, a.z / b);
}

__device__ float dot(const float3 &a, const float3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross(const float3 &a, const float3 &b)
{
    return make_float3(a.y * b.z - a.z * b.y,
                       a.z * b.x - a.x * b.z,
                       a.x * b.y - a.y * b.x);
}

__global__ void mvc_mesh_kernel(const float3 *__restrict__ points,
                                float3 *__restrict__ result,
                                const Face_Vertices *__restrict__ from_vs,
                                const Face_Vertices *__restrict__ to_vs,
                                const int point_count,
                                const int face_count)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= point_count)
    {
        return;
    }
    const float3 point = points[idx];
    float total_w = 0;
    float3 total_f = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < face_count; i++)
    {
        const auto from_t1 = from_vs[i].v0;
        const auto from_t2 = from_vs[i].v1;
        const auto from_t3 = from_vs[i].v2;
        const auto to_t1 = to_vs[i].v0;
        const auto to_t2 = to_vs[i].v1;
        const auto to_t3 = to_vs[i].v2;
        const auto v1 = from_t1 - point;
        const auto v2 = from_t2 - point;
        const auto v3 = from_t3 - point;
        const auto u1 = v1 / norm3df(v1.x, v1.y, v1.z);
        const auto u2 = v2 / norm3df(v2.x, v2.y, v2.z);
        const auto u3 = v3 / norm3df(v3.x, v3.y, v3.z);
        const auto u23 = u2 - u3;
        const auto u31 = u3 - u1;
        const auto u12 = u1 - u2;
        const auto l1 = norm3df(u23.x, u23.y, u23.z);
        const auto l2 = norm3df(u31.x, u31.y, u31.z);
        const auto l3 = norm3df(u12.x, u12.y, u12.z);
        const auto theta1 = 2 * asinf(min(l1 / 2, 1.0));
        const auto theta2 = 2 * asinf(min(l2 / 2, 1.0));
        const auto theta3 = 2 * asinf(min(l3 / 2, 1.0));
        const auto h = (theta1 + theta2 + theta3) / 2;
        if (abs(pi - h) < eps)
        {
            const auto w1 = (tanf(theta2 / 2) + tanf(theta3 / 2)) / norm3df(v1.x, v1.y, v1.z);
            const auto w2 = (tanf(theta3 / 2) + tanf(theta1 / 2)) / norm3df(v2.x, v2.y, v2.z);
            const auto w3 = (tanf(theta1 / 2) + tanf(theta2 / 2)) / norm3df(v3.x, v3.y, v3.z);
            total_w = w1 + w2 + w3;
            total_f = w1 * to_t1 + w2 * to_t2 + w3 * to_t3;
            result[idx] = total_f / total_w;
            return;
        }
        else
        {
            auto n1 = cross(u2, u3);
            n1 = n1 * rnorm3df(n1.x, n1.y, n1.z);
            auto n2 = cross(u3, u1);
            n2 = n2 * rnorm3df(n2.x, n2.y, n2.z);
            auto n3 = cross(u1, u2);
            n3 = n3 * rnorm3df(n3.x, n3.y, n3.z);
            const auto det = dot(u1, cross(u2, u3));
            if (abs(det) < eps)
            {
                continue;
            }
            const auto m = (theta1 * n1 + theta2 * n2 + theta3 * n3) / 2;
            const auto w1 = dot(n1, m) / dot(n1, v1);
            const auto w2 = dot(n2, m) / dot(n2, v2);
            const auto w3 = dot(n3, m) / dot(n3, v3);
            total_w += w1 + w2 + w3;
            total_f = total_f + w1 * to_t1 + w2 * to_t2 + w3 * to_t3;
        }
    }
    result[idx] = total_f / total_w;
    return;
}

void test_mvc_mesh(const std::string &filename,
                   const std::string &cage_filename,
                   const std::string &deformed_cage_filename,
                   const std::string &output_filename)
{
    const auto start = std::chrono::high_resolution_clock::now();

    // read data/QMVC/FireHydrant.obj
    std::vector<std::array<float, 3>> vs;
    std::vector<std::vector<int>> fs;
    if (!read_any_obj(filename, vs, fs))
        return;
    // read cage test/FireHydrant_Cage_Triangulated.obj
    std::vector<std::array<float, 3>> from_vs;
    std::vector<std::array<int, 3>> from_fs;
    if (!read_tri_obj(cage_filename, from_vs, from_fs))
        return;
    // read defromed cage test/FireHydrant_Cage_Deformed_Triangulated.obj
    std::vector<std::array<float, 3>> to_vs;
    std::vector<std::array<int, 3>> to_fs;
    if (!read_tri_obj(deformed_cage_filename, to_vs, to_fs))
        return;

    const auto read = std::chrono::high_resolution_clock::now();

    const int vertex_count = vs.size();
    std::vector<float3> vs_f3(vertex_count);
    for (int i = 0; i < vertex_count; i++)
    {
        vs_f3[i] = make_float3(vs[i][0], vs[i][1], vs[i][2]);
    }
    const int face_count = from_fs.size();
    std::vector<Face_Vertices> from_face_vs(face_count);
    for (int i = 0; i < face_count; i++)
    {
        from_face_vs[i].v0 = make_float3(from_vs[from_fs[i][0]][0], from_vs[from_fs[i][0]][1], from_vs[from_fs[i][0]][2]);
        from_face_vs[i].v1 = make_float3(from_vs[from_fs[i][1]][0], from_vs[from_fs[i][1]][1], from_vs[from_fs[i][1]][2]);
        from_face_vs[i].v2 = make_float3(from_vs[from_fs[i][2]][0], from_vs[from_fs[i][2]][1], from_vs[from_fs[i][2]][2]);
    }
    std::vector<Face_Vertices> to_face_vs(face_count);
    for (int i = 0; i < face_count; i++)
    {
        to_face_vs[i].v0 = make_float3(to_vs[to_fs[i][0]][0], to_vs[to_fs[i][0]][1], to_vs[to_fs[i][0]][2]);
        to_face_vs[i].v1 = make_float3(to_vs[to_fs[i][1]][0], to_vs[to_fs[i][1]][1], to_vs[to_fs[i][1]][2]);
        to_face_vs[i].v2 = make_float3(to_vs[to_fs[i][2]][0], to_vs[to_fs[i][2]][1], to_vs[to_fs[i][2]][2]);
    }

    cudaDeviceSynchronize();
    const auto prepare = std::chrono::high_resolution_clock::now();

    float3 *d_vs;
    cudaMalloc(&d_vs, vertex_count * sizeof(float3));
    cudaMemcpy(d_vs, vs_f3.data(), vertex_count * sizeof(float3), cudaMemcpyHostToDevice);

    cudaDeviceSynchronize();
    const auto copy_vs = std::chrono::high_resolution_clock::now();

    Face_Vertices *d_from_face_vs;
    cudaMalloc(&d_from_face_vs, face_count * sizeof(Face_Vertices));
    cudaMemcpy(d_from_face_vs, from_face_vs.data(), face_count * sizeof(Face_Vertices), cudaMemcpyHostToDevice);

    Face_Vertices *d_to_face_vs;
    cudaMalloc(&d_to_face_vs, face_count * sizeof(Face_Vertices));
    cudaMemcpy(d_to_face_vs, to_face_vs.data(), face_count * sizeof(Face_Vertices), cudaMemcpyHostToDevice);

    cudaDeviceSynchronize();
    const auto copy_face_vs = std::chrono::high_resolution_clock::now();

    float3 *d_result;
    cudaMalloc(&d_result, vertex_count * sizeof(float3));
    mvc_mesh_kernel<<<(vertex_count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>(d_vs, d_result, d_from_face_vs, d_to_face_vs, vertex_count, face_count);

    cudaDeviceSynchronize();
    const auto kernel = std::chrono::high_resolution_clock::now();

    std::vector<float3> result(vertex_count);
    cudaMemcpy(result.data(), d_result, vertex_count * sizeof(float3), cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
    const auto write = std::chrono::high_resolution_clock::now();

    std::vector<std::array<float, 3>> result_vs(vertex_count);
    for (int i = 0; i < vertex_count; i++)
    {
        result_vs[i] = {float(result[i].x), float(result[i].y), float(result[i].z)};
    }
    if (!write_any_obj(output_filename, result_vs, fs))
        return;

    const auto end = std::chrono::high_resolution_clock::now();

    const auto read_duration = std::chrono::duration_cast<std::chrono::microseconds>(read - start).count();
    const auto prepare_duration = std::chrono::duration_cast<std::chrono::microseconds>(prepare - read).count();
    const auto copy_vs_duration = std::chrono::duration_cast<std::chrono::microseconds>(copy_vs - prepare).count();
    const auto copy_face_vs_duration = std::chrono::duration_cast<std::chrono::microseconds>(copy_face_vs - copy_vs).count();
    const auto kernel_duration = std::chrono::duration_cast<std::chrono::microseconds>(kernel - copy_face_vs).count();
    const auto copy_result_duration = std::chrono::duration_cast<std::chrono::microseconds>(write - kernel).count();
    const auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(end - write).count();

    std::cout << "read duration: " << read_duration << " us" << std::endl;
    std::cout << "prepare duration: " << prepare_duration << " us" << std::endl;
    std::cout << "copy vs duration: " << copy_vs_duration << " us" << std::endl;
    std::cout << "copy face vs duration: " << copy_face_vs_duration << " us" << std::endl;
    std::cout << "kernel duration: " << kernel_duration << " us" << std::endl;
    std::cout << "copy result duration: " << copy_result_duration << " us" << std::endl;
    std::cout << "write duration: " << write_duration << " us" << std::endl;

    cudaFree(d_vs);
    cudaFree(d_result);
    cudaFree(d_from_face_vs);
    cudaFree(d_to_face_vs);
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " <filename> <cage_filename> <deformed_cage_filename> <output_filename>" << std::endl;
        return -1;
    }
    const std::string filename = argv[1];
    const std::string cage_filename = argv[2];
    const std::string deformed_cage_filename = argv[3];
    const std::string output_filename = argv[4];

    test_mvc_mesh(filename, cage_filename, deformed_cage_filename, output_filename);
    return 0;
}