class Playtime.Skins.Circle extends createjs.Shape

  defaultOptions:
    fillColor: createjs.Graphics.getRGB(0,255,0)
    radius: 40
    x: 0
    y: 0

  constructor: (args={}) ->
    super
    _.defaults(args, @defaultOptions)
    _.each args, (value, key) => @[key] = args[key]

    @graphics.beginFill(@fillColor)
    @graphics.drawCircle(@x, @y, @radius)

