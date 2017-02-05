cc      = "gcc"
inc_dir = "./"

task :default => :build

task :build do
  sh "#{cc} -I #{inc_dir} main.c i2c.c utils.c -o bme280"
end
