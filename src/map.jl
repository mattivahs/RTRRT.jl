include("obstacles.jl")

mutable struct PlanningMap
    x_range::Vector{Float64}
    y_range::Vector{Float64}
    obstacle_grid::Matrix{Vector{Any}}
    all_obstacles::Vector{Any}
    params::parameters

    function PlanningMap(params::parameters)
        empty_obstacle_grid = Matrix{Vector{Any}}(undef, params.obstacle_grid_n, params.obstacle_grid_n)
        for i = 1:params.obstacle_grid_n, j = 1:params.obstacle_grid_n
            empty_obstacle_grid[i, j] = []
        end
        new(params.x_range, params.y_range, empty_obstacle_grid, [], params)
    end
end

function add_obstacle(this::PlanningMap, obst::obstacle)
    if !(obst in this.all_obstacles)
        push!(this.all_obstacles, obst)
    end
    for ids in obst.occupied_cells
        if !(obst in this.obstacle_grid[ids[1], ids[2]])
            push!(this.obstacle_grid[ids[1], ids[2]], obst)
        end
    end
end

function load_default_map(this::PlanningMap, params::parameters)
    obstacles = Vector{obstacle}()
    line_obst1 = line_obstacle(1, [5.1, 0.0], [5.15, 5.0], params)
    push!(obstacles, line_obst1)
    line_obst2 = line_obstacle(2, [3.0, 5.0], [9.0, 5.0], params)
    push!(obstacles, line_obst2)
    poly1 = convex_polygon(3, [[3.0, 8.0], [5.0, 9.0], [8.0, 7.5], [4.0, 6.5]], params)
    push!(obstacles, poly1)
    poly2 = convex_polygon(4, [[2.0, 1.5], [2.1, 2.6], [4.0, 3.0], [2.5, 2.0], [4.0, 1.0]], params)
    push!(obstacles, poly2)
    for obst in obstacles
        add_obstacle(this, obst)
    end
end

function visualize_map(this::PlanningMap)
    for obst in this.all_obstacles
        plot_obstacle(obst)
    end
end

function edge_collision(this::PlanningMap, p1, p2)
    obstacles_checked = []
    # Intersected cells of obstacle grid
    cells = intersected_cells([p1, p2], this.params)
    for cell_id in cells
        for obst in this.obstacle_grid[cell_id[1], cell_id[2]]
            if !(obst.id in obstacles_checked)
                push!(obstacles_checked, obst.id)
                if collision_check(p1, p2, obst)
                    return true
                end
            end
        end
    end
    return false
end
