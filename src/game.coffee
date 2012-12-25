class Game
  constructor: ->
    $canvas = $('canvas')
    @context = $canvas[0].getContext('2d')
    @width = $canvas.width()
    @height = $canvas.height()
    @setupKeys()

  rectX: 100
  rectY: 100

  setupKeys: =>
    key 'up, down, left, right', @moveKeys

  moveKeys: =>
    keys = key.getPressedKeyCodes()

    moveSpeed = 2
    dy = 0
    dx = 0

    _.each keys, (key) =>
      switch key
        when 37 #left
          dx -= moveSpeed
        when 38 #up
          dy -= moveSpeed
        when 39 #right
          dx += moveSpeed
        when 40 #down
          dy += moveSpeed
        else
          console.log 'unknown key', key

    @moveRect(dy, dx)

  moveRect: (dy=0, dx=0) =>
    @clear()
    @rectX += dx
    @rectY += dy
    @drawRect(@rectX, @rectY, 20, 20)

  colors:
    green: "#33A534"
    black: "#000000"
    red: "#FF2600"

  drawCircle: (x, y, r, fillColor = @colors.green) =>
    @context.fillStyle = fillColor
    @context.beginPath()
    @context.arc(x, y, r, 0, 2 * Math.Pi, true)
    @context.closePath()
    @context.fill()

  clear: =>
    @context.clearRect(0, 0, @width, @height)

  draw: =>
    @clear()
    @drawRect(@rectX, @rectY, 20, 20)

  drawRect: (x, y, width, height, fillColor = @colors.red) =>
    @context.fillStyle = fillColor
    @context.beginPath()
    @context.rect(x, y, width, height)
    @context.closePath()
    @context.fill()

