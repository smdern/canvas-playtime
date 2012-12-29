class Sprite
  defaultOptions:
    name: null
    painter: null
    top: 0
    left: 0
    height: 10
    width: 10
    velocityX: 0
    velocityY: 0
    visible: true
    animating: false
    behaviors: []

  constructor: (args) ->
    _.defaults(args, @defaultOptions)
    _.each args, (value, key) => @[key] = args[key]

  paint: (context) => @painter.paint(@, context) if @painter && @visible

  update: (context, time) => _.each @behaviors, (behavior) => behavior.execute(@, context, time)

  getBoundingBox: =>
    top: @top
    left: @left
    height: @height
    width: @width

