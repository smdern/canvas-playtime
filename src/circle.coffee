class Circle
  defaultOptions:
    x: 10
    y: 10
    radius: 20
    startAngle: 0
    endAngle: 2 * Math.PI
    antiClockwise: true
    fillStyle: 'black'

  constructor: (args) ->
    _.defaults(args, @defaultOptions)
    _.each ['x', 'y', 'radius', 'startAngle', 'endAngle', 'antiClockwise', 'fillStyle'], (option) =>
      @[option] = args[option]

  draw: (context, dx=0, dy=0) =>
    @x += dx
    @y += dy
    context.fillStyle = @fillStyle
    context.lineWidth = 5
    context.beginPath()
    context.arc(@x, @y, @radius, @startAngle, @endAngle, @antiClockwise)
    context.stroke()
    context.closePath()
    context.fill()

