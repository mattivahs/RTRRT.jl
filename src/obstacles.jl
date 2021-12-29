include("helper_functions.jl")
using GeometryTypes

abstract type obstacle end

struct line_obstacle <: obstacle
    id::Int64
    p1::Vector{Float64}
    p2::Vector{Float64}
    points::Vector{Vector{Float64}}
    occupied_cells::Vector{Vector{Int64}}
    lines::Vector{LineSegment{Point{2, Float32}}}

    function line_obstacle(id::Int64, p1::Vector{Float64}, p2::Vector{Float64}, params::parameters)
        occupied_cells = intersected_cells([p1, p2], params)
        lines = LineSegment{Point{2, Float32}}[]
        points = Vector{Vector{Float64}}()
        push!(points, p1)
        push!(points, p2)
        push!(lines, LineSegment(Point2f0(p1[1], p1[2]), Point2f0(p2[1], p2[2])))
        new(id, p1, p2, points, occupied_cells, lines)
    end
end

struct convex_polygon <: obstacle
    id::Int64
    # Points are defined such that clockwise connection defines the polygon
    points::Vector{Vector{Float64}}
    occupied_cells::Vector{Vector{Int64}}
    lines::Vector{LineSegment{Point{2, Float32}}}

    function convex_polygon(id::Int64, points::Vector{Vector{Float64}}, params::parameters)
        length(points) < 3 ? throw(DomainError("Polygon must have at least 3 points")) : nothing
        occupied_cells = intersected_cells(points, params)
        lines = LineSegment{Point{2, Float32}}[]
        for i = 1:(length(points)-1)
            push!(lines, LineSegment(Point2f0(points[i][1], points[i][2]), Point2f0(points[i+1][1], points[i+1][2])))
        end
        push!(lines, LineSegment(Point2f0(points[end][1], points[end][2]), Point2f0(points[1][1], points[1][2])))
        new(id, points, occupied_cells, lines)
    end
end

function collision_check(p1::Vector{Float64}, p2::Vector{Float64}, this::T) where {T <: obstacle}
    edge = LineSegment(Point2f0(p1[1], p1[2]), Point2f0(p2[1], p2[2]))
    for line in this.lines
        if intersects(edge, line)[1]
            return true
        end
    end
    return false
end

function plot_obstacle(this::T) where {T <: obstacle}
    x_data = []
    y_data = []
    for pt in this.points
        push!(x_data, pt[1])
        push!(y_data, pt[2])
    end
    push!(x_data, this.points[1][1])
    push!(y_data, this.points[1][2])

    plot!(x_data, y_data, legend=false, color="blue")
end
