#ifndef MESH_IO_H
#define MESH_IO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>

inline bool read_tri_obj(const std::string &filename,
                  std::vector<std::array<float, 3>> &vertices,
                  std::vector<std::array<int, 3>> &faces)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    for (std::string line; std::getline(file, line);)
    {
        if (line[0] == 'v')
        {
            std::array<float, 3> vertex;
            if (line[1] == ' ')
            {
                std::stringstream ss(line.substr(2));
                ss >> vertex[0] >> vertex[1] >> vertex[2];
                vertices.push_back(vertex);
            }
        }
        else if (line[0] == 'f')
        {
            std::array<int, 3> face;
            if (line[1] == ' ')
            {
                int v0, v1, v2;
                std::stringstream ss(line.substr(2));
                ss >> v0 >> v1 >> v2;
                face[0] = v0 - 1;
                face[1] = v1 - 1;
                face[2] = v2 - 1;
                faces.push_back(face);
            }
        }
    }
    return true;
}

inline bool read_tri_off(const std::string &filename,
                  std::vector<std::array<float, 3>> &vertices,
                  std::vector<std::array<int, 3>> &faces)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    std::string tag;
    file >> tag;
    if (tag != "OFF")
    {
        std::cerr << "File is not in OFF format" << std::endl;
        return false;
    }
    int num_vertices, num_faces, num_edges;
    file >> num_vertices >> num_faces >> num_edges;
    vertices.resize(num_vertices);
    faces.resize(num_faces);
    for (int i = 0; i < num_vertices; i++)
    {
        file >> vertices[i][0] >> vertices[i][1] >> vertices[i][2];
    }
    for (int i = 0; i < num_faces; i++)
    {
        int n;
        file >> n;
        if (n != 3)
        {
            std::cerr << "Only triangular faces are supported" << std::endl;
            return false;
        }
        file >> faces[i][0] >> faces[i][1] >> faces[i][2];
    }
    return true;
}

inline bool write_tri_off(const std::string &filename,
                   const std::vector<std::array<float, 3>> &vertices,
                   const std::vector<std::array<int, 3>> &faces)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    file << "OFF" << std::endl;
    file << vertices.size() << " " << faces.size() << " 0" << std::endl;
    for (const auto &vertex : vertices)
    {
        file << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    for (const auto &face : faces)
    {
        file << "3 " << face[0] << " " << face[1] << " " << face[2] << std::endl;
    }
    return true;
}

inline bool write_tri_obj(const std::string &filename,
                   const std::vector<std::array<float, 3>> &vertices,
                   const std::vector<std::array<int, 3>> &faces)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    for (const auto &vertex : vertices)
    {
        file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    for (const auto &face : faces)
    {
        file << "f " << (face[0] + 1) << " " << (face[1] + 1) << " " << (face[2] + 1) << std::endl;
    }
    return true;
}

inline bool read_any_obj(const std::string &filename,
                  std::vector<std::array<float, 3>> &vertices,
                  std::vector<std::vector<int>> &faces)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    for (std::string line; std::getline(file, line);)
    {
        if (line[0] == 'v')
        {
            std::array<float, 3> vertex;
            if (line[1] == ' ')
            {
                std::stringstream ss(line.substr(2));
                ss >> vertex[0] >> vertex[1] >> vertex[2];
                vertices.push_back(vertex);
            }
        }
        else if (line[0] == 'f')
        {
            std::vector<int> face;
            std::string face_str = line.substr(2);
            std::stringstream ss(face_str);
            int v;
            while (ss >> v)
            {
                face.push_back(v - 1);
            }
            faces.push_back(face);
        }
    }
    return true;
}

inline bool read_any_off(const std::string &filename,
                  std::vector<std::array<float, 3>> &vertices,
                  std::vector<std::vector<int>> &faces)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    std::string tag;
    file >> tag;
    if (tag != "OFF")
    {
        std::cerr << "File is not in OFF format" << std::endl;
        return false;
    }
    int num_vertices, num_faces, num_edges;
    file >> num_vertices >> num_faces >> num_edges;
    vertices.resize(num_vertices);
    faces.resize(num_faces);
    for (int i = 0; i < num_vertices; i++)
    {
        file >> vertices[i][0] >> vertices[i][1] >> vertices[i][2];
    }
    for (int i = 0; i < num_faces; i++)
    {
        int n;
        file >> n;
        std::vector<int> face;
        for (int j = 0; j < n; j++)
        {
            int vertex_index;
            file >> vertex_index;
            face.push_back(vertex_index);
        }
        faces.push_back(face);
    }
    return true;
}

inline bool write_any_off(const std::string &filename,
                   const std::vector<std::array<float, 3>> &vertices,
                   const std::vector<std::vector<int>> &faces)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    file << "OFF" << std::endl;
    file << vertices.size() << " " << faces.size() << " 0" << std::endl;
    for (const auto &vertex : vertices)
    {
        file << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    for (const auto &face : faces)
    {
        file << face.size();
        for (const auto &vertex_index : face)
        {
            file << " " << vertex_index;
        }
        file << std::endl;
    }
    return true;
}

inline bool write_any_obj(const std::string &filename,
                   const std::vector<std::array<float, 3>> &vertices,
                   const std::vector<std::vector<int>> &faces)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << filename << std::endl;
        return false;
    }
    for (const auto &vertex : vertices)
    {
        file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    for (const auto &face : faces)
    {
        file << "f";
        for (const auto &vertex_index : face)
        {
            file << " " << (vertex_index + 1);
        }
        file << std::endl;
    }
    return true;
}

#endif // MESH_IO_H