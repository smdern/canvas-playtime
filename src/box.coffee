class Box

  defaultOptions:
    x: 0
    y: 0
    width: 10
    height: 10
    fillStyle: 'black'

  constructor: (args) ->
    _.defaults(args, @defaultOptions)
    _.each ['x', 'y', 'width', 'height', 'fillStyle'], (option) =>
      @[option] = args[option]

  draw: (context) =>
    context.fillStyle = @fillStyle
    context.beginPath()
    context.rect(@x, @y, @width, @height)
    context.closePath()
    context.fill()

