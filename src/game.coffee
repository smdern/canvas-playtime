class Game

  isRunning: false

  defaultBall:
    painter: new CirclePainter()
    velocityY: 1
    velocityX: 3
    width: 20
    top: 10
    left: 20
    behaviors: [Behaviors.moveBall]


  constructor: ->
    $canvas = $('canvas')
    @context = $canvas[0].getContext('2d')
    @width = $canvas.width()
    @height = $canvas.height()
    $canvas.mousedown(@spawnBall)
    @ballSprites = []
    @ballSprites.push new Sprite(@defaultBall)

  spawnBall: =>
    console.log 'spawning ball'
    @ballSprites.push new Sprite(@defaultBall)

  start: =>
    @isRunning = true
    requestNextAnimationFrame.call(window, @animate)

  stop: => @isRunning = false

  prevTime: Date.now()

  animate: (time) =>
    dt = (time - @prevTime) / 10
    @prevTime = time
    if @isRunning
      @clear()
      @update(dt)
      @draw()
    requestNextAnimationFrame.call(window, @animate)

  clear: => @context.clearRect(0, 0, @width, @height)

  update: (time) =>
    @handleEdgeCollisions()
    _.each @ballSprites, (ballSprite) =>
      ballSprite.update(@context, time)

  draw: => _.each @ballSprites, (ballSprite) => ballSprite.paint(@context)

  handleEdgeCollisions: =>
    _.each @ballSprites, (ballSprite) =>
      boundingBox = ballSprite.getBoundingBox()
      right = boundingBox.left + boundingBox.width
      bottom = boundingBox.top + boundingBox.height

      ballSprite.velocityX *= -1 if (right > @width || boundingBox.left < 0)
      ballSprite.velocityY *= -1 if (bottom > @height || boundingBox.top < 0)

