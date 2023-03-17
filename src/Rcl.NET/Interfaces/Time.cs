namespace Rosidl.Messages.Builtin;

partial class Time
{
    public unsafe partial struct Priv
    {
        /// <summary>
        /// Copy value from specified <see cref="DateTimeOffset"/>.
        /// </summary>
        /// <param name="time"></param>
        public void CopyFrom(DateTimeOffset time)
            => CopyFrom(time - DateTimeOffset.UnixEpoch);

        /// <summary>
        /// Copy value from specified <see cref="TimeSpan"/>.
        /// </summary>
        /// <param name="time"></param>
        public void CopyFrom(TimeSpan time)
        {
            Sec = (int)(time.TotalNanoseconds / 1000000000);
            Nanosec = (uint)(time.TotalNanoseconds % 1000000000);
        }

        public static implicit operator TimeSpan(Priv priv)
            => TimeSpan.FromSeconds(priv.Sec + priv.Nanosec / 1000000000.0);
    }
}
