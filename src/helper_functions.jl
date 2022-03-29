using LinearAlgebra
using Plots

struct parameters
    gamma::Float64
    x_range::Vector{Float64}
    y_range::Vector{Float64}
    max_tree_nodes::Int64
    nominal_step::Float64
    expansion_time::Float64
    RewireTime::Float64
    obstacle_grid_n::Int64
    cell_size_x::Float64
    cell_size_y::Float64
    function parameters(gamma::Float64, x_range::Vector{Float64}, y_range::Vector{Float64},
        max_tree_nodes::Int64, nominal_step::Float64, expansion_time::Float64,
        RewireTime::Float64, obstacle_grid_n::Int64)
        cell_size_x = diff(x_range)[1] / obstacle_grid_n
        cell_size_y = diff(y_range)[1] / obstacle_grid_n
        new(gamma, x_range, y_range, max_tree_nodes, nominal_step, expansion_time,
            RewireTime, obstacle_grid_n, cell_size_x, cell_size_y)
    end
end

function calc_edge_cost(p1::Vector{Float64}, p2::Vector{Float64})
    "Euclidean distance edge cost"
    return norm(p2[1:2] - p1[1:2])
end

function sample_point(parameters)
    "Sample uniformly in workspace"
    x = parameters.x_range[1] + (parameters.x_range[2] - parameters.x_range[1]) * rand()
    y = parameters.y_range[1] + (parameters.y_range[2] - parameters.y_range[1]) * rand()
    return [x, y]
end

function plot_tree(tree)
    x_data = []
    y_data = []

    for node in tree
        if node.parent_id != 0
            push!(x_data, [tree[node.parent_id].pose[1], node.pose[1]])
            push!(y_data, [tree[node.parent_id].pose[2], node.pose[2]])
        end
    end
    plot!(x_data, y_data, marker=(:circle,2), legend=false, color="grey")
    plot!([tree[1].pose[1]], [tree[1].pose[2]], marker=(:circle,5), legend=false, color="red")
end

function plot_goal_path(goal_path)
    x_data = []
    y_data = []
    for i = 1:(length(goal_path) - 1)
        push!(x_data, [goal_path[i][1], goal_path[i+1][1]])
        push!(y_data, [goal_path[i][2], goal_path[i+1][2]])
    end
    plot!(x_data, y_data, marker=(:circle,2), legend=false, color="red")
end

function intersected_cells(points, params::parameters)
    # Min point p1, max point x2
    min_x = params.x_range[2]
    max_x = params.x_range[1]
    min_y = params.y_range[2]
    max_y = params.y_range[1]

    for pt in points
        pt[1] < min_x ? min_x = pt[1] : nothing
        pt[1] > max_x ? max_x = pt[1] : nothing
        pt[2] < min_y ? min_y = pt[2] : nothing
        pt[2] > max_y ? max_y = pt[2] : nothing
    end

    # Get indices of occ cells in obstacle grid
    id_x_min = 1 + clamp(floor(Int64, (min_x - params.x_range[1]) / params.cell_size_x), 0, params.obstacle_grid_n - 1);
    id_y_min = 1 + clamp(floor(Int64, (min_y - params.y_range[1]) / params.cell_size_y), 0, params.obstacle_grid_n - 1);
    id_x_max = 1 + clamp(floor(Int64, (max_x - params.x_range[1]) / params.cell_size_x), 0, params.obstacle_grid_n - 1);
    id_y_max = 1 + clamp(floor(Int64, (max_y - params.y_range[1]) / params.cell_size_y), 0, params.obstacle_grid_n - 1);

    occ_cells = Vector{Vector{Int64}}()
    for i = id_x_min:id_x_max, j = id_y_min:id_y_max
        push!(occ_cells, [i, j])
    end
    return occ_cells
end

function ConstructInequalitiesFromPoints(points)
    # Points need to be defined in clockwise order
    A = zeros(length(points), 2)
    b = zeros(length(points))

    for i = 1:(length(points))
        if i == length(points)
            j = 1
        else
            j = i + 1
        end

        p1 = points[i]
        p2 = points[j]

        dx = p2[1] - p1[1]
        if abs(dx) <= 0.0001
            j == length(points) ? next_ind = 1 : next_ind = j + 1
            next_dx = points[next_ind][1] - p2[1]
            A[i, 1], A[i, 2] = 1, 0
            b[i] = p1[1]
            if next_dx > 0
                A[i, 1] *= -1
                b[i] *= -1
            end
        else
            A[i, 2] = 1
            A[i, 1] = -(p2[2] - p1[2]) / (p2[1] - p1[1])
            b[i] = (A[i, 1] * p1[1] + p1[2])
        end
        if dx < 0 && abs(dx) > 0.0001
            A[i, 2] *= -1
            A[i, 1] *= -1
            b[i] *= -1
        end
    end
    return A, b
end
