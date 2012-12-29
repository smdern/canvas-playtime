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
    animating: true
    behaviors: []

  constructor: (args) ->
    _.defaults(args, @defaultOptions)
    _.each args, (value, key) => @[key] = args[key]

  paint: (context) => @painter.paint(@, context) if @painter && @visible

  update: (context, time) => (_.each @behaviors, (behavior) => behavior.execute(@, context, time)) if @animating

  getBoundingBox: =>
    top: @top
    left: @left
    height: @height
    width: @width

