#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <chrono>

#include "mesh_io.hpp"

using Point = std::array<float, 3>;

struct Face_Vertices
{
    float3 v0;
    float3 v1;
    float3 v2;
};

struct Mesh_By_vs_fs
{
    std::vector<Point> vs;
    std::vector<std::array<int, 3>> fs;
};

struct Mesh_and_Weights
{
    Mesh_By_vs_fs mesh;
    std::vector<std::vector<float>> all_weights;
};

const std::string ngon_prefix = "./ngon/";

int load_ngons(const std::vector<std::vector<int>> &fs,
               std::unordered_map<int, Mesh_and_Weights> &ngons)
{
    int total_face_count = 0;
    for (const auto f : fs)
    {
        const auto vertex_count = f.size();
        if (vertex_count < 3)
        {
            std::cerr << "Invalid face with less than 3 vertices:" << f[0] << " " << f[1] << " " << f[2] << std::endl;
            return -1;
        }
        if (ngons.find(vertex_count) == ngons.end())
        {
            const auto ngon_off_path = ngon_prefix + std::to_string(vertex_count) + "gon.off";
            const auto ngon_coord_path = ngon_prefix + std::to_string(vertex_count) + "gon.coord";
            Mesh_By_vs_fs mesh;
            read_tri_off(ngon_off_path, mesh.vs, mesh.fs);
            std::vector<std::vector<float>> all_weights;
            std::ifstream coord_file(ngon_coord_path);
            for (int i = 0; i < mesh.vs.size(); i++)
            {
                std::vector<float> weights;
                for (int j = 0; j < vertex_count; j++)
                {
                    float weight;
                    coord_file >> weight;
                    weights.push_back(weight);
                }
                all_weights.push_back(weights);
            }
            ngons[vertex_count] = {mesh, all_weights};
        }
        total_face_count += ngons.at(vertex_count).mesh.fs.size();
    }
    return total_face_count;
}

void triangulate_mesh(const std::vector<Point> &vs,
                      const std::vector<std::vector<int>> &fs,
                      std::vector<Face_Vertices> &result)
{
    std::unordered_map<int, Mesh_and_Weights> ngons;
    const auto total_face_count = load_ngons(fs, ngons);
    if (total_face_count < 0)
    {
        std::cerr << "Failed to load ngons" << std::endl;
        return;
    }
    result.clear();
    result.reserve(total_face_count);
    for (const auto f : fs)
    {
        const auto vertex_count = f.size();
        if (vertex_count < 3)
        {
            std::cerr << "Invalid face with less than 3 vertices" << std::endl;
            return;
        }
        {
            const auto &mesh = ngons.at(vertex_count).mesh;
            const auto &all_weights = ngons.at(vertex_count).all_weights;
            std::vector<Point> f_vertices;
            for (const auto v_index : f)
            {
                f_vertices.push_back(vs[v_index]);
            }
            std::vector<Point> new_vs(mesh.vs.size());
            for (int i = 0; i < mesh.vs.size(); i++)
            {
                Point new_v = {0, 0, 0};
                for (int j = 0; j < vertex_count; j++)
                {
                    new_v[0] += all_weights[i][j] * f_vertices[j][0];
                    new_v[1] += all_weights[i][j] * f_vertices[j][1];
                    new_v[2] += all_weights[i][j] * f_vertices[j][2];
                }
                new_vs[i] = new_v;
            }
            for (const auto &face : mesh.fs)
            {
                Face_Vertices face_vertices = {
                    make_float3(new_vs[face[0]][0], new_vs[face[0]][1], new_vs[face[0]][2]),
                    make_float3(new_vs[face[1]][0], new_vs[face[1]][1], new_vs[face[1]][2]),
                    make_float3(new_vs[face[2]][0], new_vs[face[2]][1], new_vs[face[2]][2]),
                };
                result.push_back(face_vertices);
            }
        }
    }
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " input_filename output_filename" << std::endl;
        return -1;
    }
    const std::string input_path = argv[1];
    const std::string output_path = argv[2];

    const auto start = std::chrono::high_resolution_clock::now();

    std::vector<Point> vs;
    std::vector<std::vector<int>> fs;
    if (!read_any_obj(input_path, vs, fs))
        return -1;
    const auto read = std::chrono::high_resolution_clock::now();

    std::vector<Face_Vertices> result;
    triangulate_mesh(vs, fs, result);
    const auto triangulation = std::chrono::high_resolution_clock::now();

    const auto face_count = result.size();
    const auto vertex_count = face_count * 3;
    std::vector<Point> new_vs;
    new_vs.reserve(vertex_count);
    std::vector<std::array<int, 3>> new_fs;
    new_fs.reserve(face_count);
    int index = 0;
    for (const auto &face_vertices : result)
    {
        new_fs.push_back({index, index + 1, index + 2});
        new_vs.push_back({face_vertices.v0.x, face_vertices.v0.y, face_vertices.v0.z});
        new_vs.push_back({face_vertices.v1.x, face_vertices.v1.y, face_vertices.v1.z});
        new_vs.push_back({face_vertices.v2.x, face_vertices.v2.y, face_vertices.v2.z});
        index += 3;
    }
    write_tri_obj(output_path, new_vs, new_fs);
    const auto write = std::chrono::high_resolution_clock::now();

    const auto read_duration = std::chrono::duration_cast<std::chrono::microseconds>(read - start).count();
    const auto triangulation_duration = std::chrono::duration_cast<std::chrono::microseconds>(triangulation - read).count();
    const auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(write - triangulation).count();
    const auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(write - start).count();
    std::cout << "Read: " << read_duration << "us" << std::endl;
    std::cout << "Triangulation: " << triangulation_duration << "us" << std::endl;
    std::cout << "Write: " << write_duration << "us" << std::endl;
    std::cout << "Total: " << total_duration << "us" << std::endl;
    return 0;
}